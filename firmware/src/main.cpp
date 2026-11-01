#include <array>
#include <cstdint>
#include <optional>
#include <span>

#include "bridge_controller.hpp"
#include "hardware/watchdog.h"
#include "motor_protocol.hpp"
#include "pc_protocol.hpp"
#include "pico/stdlib.h"
#include "xl2515.hpp"

namespace {

using pc_protocol::ErrorCode;
using pc_protocol::MessageType;

constexpr std::uint32_t HARDWARE_WATCHDOG_MS = 100;
constexpr std::uint32_t STATUS_PERIOD_MS = 100;
constexpr std::uint32_t CAN_TRANSMIT_TIMEOUT_MS = 100;
constexpr std::size_t CAN_QUEUE_CAPACITY = 8;
constexpr std::size_t SET_VELOCITY_PAYLOAD_SIZE = 4;
constexpr int MAX_RECEIVE_PER_LOOP = 4;
constexpr int MAX_PC_BYTES_PER_LOOP = 256;
constexpr uint LED_PIN = 25;

// Velocity targets keep only the newest value, so a stale non-zero target can never
// be sent late. Settings stay ordered, and a disable supersedes anything still queued.
class CanTransmitQueue {
public:
    [[nodiscard]] bool enqueue(const bridge::Output& output) {
        bool hasRoom = true;
        for (std::size_t index = 0; index < output.count; ++index) {
            hasRoom = enqueueFrame(output.frames[index]) && hasRoom;
        }
        return hasRoom;
    }

    // Returns true when a stuck frame had to be aborted (nothing acknowledged it).
    [[nodiscard]] bool service(std::uint32_t nowMs) {
        if (isSending_) {
            if (!xl2515::isTransmitBusy()) {
                isSending_ = false;
                isAborting_ = false;
            } else if (!isAborting_ && nowMs - sendStartMs_ >= CAN_TRANSMIT_TIMEOUT_MS) {
                xl2515::abortTransmit();
                isAborting_ = true;
                return true;
            } else {
                return false;
            }
        }
        if (latestVelocity_) {
            send(*latestVelocity_, nowMs);
            latestVelocity_.reset();
        } else if (size_ > 0) {
            send(frames_[head_], nowMs);
            head_ = (head_ + 1) % frames_.size();
            --size_;
        }
        return false;
    }

private:
    [[nodiscard]] bool enqueueFrame(const motor_protocol::CanFrame& frame) {
        if (frame.id == motor_protocol::VELOCITY_COMMAND_ID) {
            latestVelocity_ = frame;
            return true;
        }
        if (frame.id == motor_protocol::SET_MODE_ID &&
            frame.data[0] == motor_protocol::MODE_DISABLE) {
            size_ = 0;
        }
        if (size_ == frames_.size()) {
            return false;
        }
        frames_[(head_ + size_) % frames_.size()] = frame;
        ++size_;
        return true;
    }

    void send(const motor_protocol::CanFrame& frame, std::uint32_t nowMs) {
        xl2515::transmit(frame);
        isSending_ = true;
        sendStartMs_ = nowMs;
    }

    std::array<motor_protocol::CanFrame, CAN_QUEUE_CAPACITY> frames_{};
    std::optional<motor_protocol::CanFrame> latestVelocity_;
    std::size_t head_ = 0;
    std::size_t size_ = 0;
    bool isSending_ = false;
    bool isAborting_ = false;
    std::uint32_t sendStartMs_ = 0;
};

void sendToPc(MessageType type, std::span<const std::uint8_t> payload) {
    const pc_protocol::EncodedFrame encoded = pc_protocol::encode(type, payload);
    stdio_put_string(reinterpret_cast<const char*>(encoded.bytes.data()),
                     static_cast<int>(encoded.size), false, false);
}

void sendError(ErrorCode code) {
    const std::array<std::uint8_t, 1> payload{static_cast<std::uint8_t>(code)};
    sendToPc(MessageType::error, payload);
}

void sendStatus(const bridge::Controller& controller) {
    const std::array<std::uint8_t, 2> payload{static_cast<std::uint8_t>(controller.state()),
                                              static_cast<std::uint8_t>(controller.stopReason())};
    sendToPc(MessageType::status, payload);
}

void sendFeedback(const motor_protocol::Feedback& feedback) {
    const auto velocity = static_cast<std::uint16_t>(feedback.velocityRpm);
    const auto current = static_cast<std::uint16_t>(feedback.currentRaw);
    const std::array<std::uint8_t, 9> payload{
        feedback.motorId,
        static_cast<std::uint8_t>(velocity & 0xFF),
        static_cast<std::uint8_t>(velocity >> 8),
        static_cast<std::uint8_t>(current & 0xFF),
        static_cast<std::uint8_t>(current >> 8),
        static_cast<std::uint8_t>(feedback.angleRaw & 0xFF),
        static_cast<std::uint8_t>(feedback.angleRaw >> 8),
        feedback.fault,
        feedback.mode,
    };
    sendToPc(MessageType::feedback, payload);
}

void reportResult(bridge::CommandResult result) {
    if (result == bridge::CommandResult::velocityOutOfRange) {
        sendError(ErrorCode::velocityOutOfRange);
    } else if (result == bridge::CommandResult::invalidState) {
        sendError(ErrorCode::invalidState);
    }
}

void handleTransmitFailure(bridge::Controller& controller, std::uint32_t nowMs) {
    controller.onCanTransmitFailure(nowMs);
    sendError(ErrorCode::canTransmitTimeout);
}

void enqueueOutput(const bridge::Output& output, bridge::Controller& controller,
                   CanTransmitQueue& queue, std::uint32_t nowMs) {
    if (!queue.enqueue(output)) {
        handleTransmitFailure(controller, nowMs);
    }
}

std::int16_t readLittleEndianInt16(const pc_protocol::Frame& frame, std::size_t offset) {
    return static_cast<std::int16_t>(
        static_cast<std::uint16_t>(frame.payload[offset]) |
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(frame.payload[offset + 1]) << 8));
}

void handlePcFrame(const pc_protocol::Frame& frame, bridge::Controller& controller,
                   CanTransmitQueue& queue, std::uint32_t nowMs) {
    switch (static_cast<MessageType>(frame.type)) {
        case MessageType::enable: {
            bridge::Output output;
            reportResult(controller.enable(nowMs, output));
            enqueueOutput(output, controller, queue, nowMs);
            return;
        }
        case MessageType::disable:
            reportResult(controller.disable(nowMs));
            return;
        case MessageType::setVelocity:
            if (frame.payloadSize != SET_VELOCITY_PAYLOAD_SIZE) {
                sendError(ErrorCode::invalidLength);
                return;
            }
            reportResult(controller.setVelocity(readLittleEndianInt16(frame, 0),
                                                readLittleEndianInt16(frame, 2), nowMs));
            return;
        default:
            sendError(ErrorCode::unknownType);
    }
}

void pollPc(pc_protocol::StreamParser& parser, bridge::Controller& controller,
            CanTransmitQueue& queue, std::uint32_t nowMs) {
    // Bounded so that a flooding PC cannot starve the control loop.
    for (int count = 0; count < MAX_PC_BYTES_PER_LOOP; ++count) {
        const int character = stdio_getchar_timeout_us(0);
        if (character == PICO_ERROR_TIMEOUT) {
            return;
        }
        const pc_protocol::ParseResult result = parser.push(static_cast<std::uint8_t>(character));
        if (result.error) {
            sendError(*result.error);
        }
        if (result.frame) {
            handlePcFrame(*result.frame, controller, queue, nowMs);
        }
    }
}

void pollCan(bridge::Controller& controller, std::uint32_t nowMs) {
    for (int count = 0; count < MAX_RECEIVE_PER_LOOP; ++count) {
        const auto frame = xl2515::receive();
        if (!frame) {
            return;
        }
        const auto feedback = motor_protocol::parseFeedback(*frame);
        if (feedback) {
            controller.onFeedback(*feedback, nowMs);
            sendFeedback(*feedback);
        }
    }
}

// Never returns: without a working CAN controller the motors cannot be commanded at all.
[[noreturn]] void blinkForever() {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_xor_mask(1U << LED_PIN);
        sleep_ms(100);
    }
}

}  // namespace

int main() {
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);

    if (!xl2515::initialize()) {
        blinkForever();
    }

    bridge::Controller controller;
    CanTransmitQueue queue;
    pc_protocol::StreamParser parser;
    watchdog_enable(HARDWARE_WATCHDOG_MS, true);

    std::uint32_t lastStatusMs = 0;
    while (true) {
        watchdog_update();
        const std::uint32_t nowMs = to_ms_since_boot(get_absolute_time());
        pollPc(parser, controller, queue, nowMs);
        pollCan(controller, nowMs);
        enqueueOutput(controller.tick(nowMs), controller, queue, nowMs);
        if (queue.service(nowMs)) {
            handleTransmitFailure(controller, nowMs);
        }
        if (nowMs - lastStatusMs >= STATUS_PERIOD_MS) {
            lastStatusMs = nowMs;
            sendStatus(controller);
        }
    }
}
