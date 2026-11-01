#include "pc_protocol.hpp"

namespace pc_protocol {
namespace {

constexpr std::uint16_t CRC_POLYNOMIAL = 0x1021;
constexpr std::uint16_t CRC_INITIAL = 0xFFFF;

std::uint16_t updateCrc(std::uint16_t crc, std::uint8_t byte) {
    crc = static_cast<std::uint16_t>(crc ^ (static_cast<std::uint16_t>(byte) << 8));
    for (int bit = 0; bit < 8; ++bit) {
        const bool isHighBitSet = (crc & 0x8000) != 0;
        crc = static_cast<std::uint16_t>(crc << 1);
        if (isHighBitSet) {
            crc = static_cast<std::uint16_t>(crc ^ CRC_POLYNOMIAL);
        }
    }
    return crc;
}

}  // namespace

std::uint16_t crc16(std::span<const std::uint8_t> bytes) {
    std::uint16_t crc = CRC_INITIAL;
    for (const std::uint8_t byte : bytes) {
        crc = updateCrc(crc, byte);
    }
    return crc;
}

EncodedFrame encode(MessageType type, std::span<const std::uint8_t> payload) {
    EncodedFrame encoded;
    auto& bytes = encoded.bytes;
    bytes[0] = HEADER_FIRST;
    bytes[1] = HEADER_SECOND;
    bytes[2] = static_cast<std::uint8_t>(type);
    bytes[3] = static_cast<std::uint8_t>(payload.size());
    std::size_t offset = 4;
    for (const std::uint8_t byte : payload) {
        bytes[offset++] = byte;
    }
    const std::uint16_t crc =
        crc16(std::span<const std::uint8_t>(bytes.data() + 2, payload.size() + 2));
    bytes[offset++] = static_cast<std::uint8_t>(crc & 0xFF);
    bytes[offset++] = static_cast<std::uint8_t>(crc >> 8);
    encoded.size = offset;
    return encoded;
}

ParseResult StreamParser::push(std::uint8_t byte) {
    switch (step_) {
        case Step::headerFirst:
            if (byte == HEADER_FIRST) {
                step_ = Step::headerSecond;
            }
            return {};
        case Step::headerSecond:
            if (byte == HEADER_SECOND) {
                step_ = Step::type;
            } else if (byte != HEADER_FIRST) {
                step_ = Step::headerFirst;
            }
            return {};
        case Step::type:
            frame_ = Frame{};
            frame_.type = byte;
            step_ = Step::length;
            return {};
        case Step::length:
            if (byte > MAX_PAYLOAD_SIZE) {
                step_ = Step::headerFirst;
                return {.frame = std::nullopt, .error = ErrorCode::invalidLength};
            }
            frame_.payloadSize = byte;
            payloadIndex_ = 0;
            step_ = byte == 0 ? Step::crcLow : Step::payload;
            return {};
        case Step::payload:
            frame_.payload[payloadIndex_++] = byte;
            if (payloadIndex_ == frame_.payloadSize) {
                step_ = Step::crcLow;
            }
            return {};
        case Step::crcLow:
            crcLow_ = byte;
            step_ = Step::crcHigh;
            return {};
        case Step::crcHigh: {
            step_ = Step::headerFirst;
            std::array<std::uint8_t, MAX_PAYLOAD_SIZE + 2> checked{};
            checked[0] = frame_.type;
            checked[1] = frame_.payloadSize;
            for (std::size_t index = 0; index < frame_.payloadSize; ++index) {
                checked[index + 2] = frame_.payload[index];
            }
            const std::uint16_t received =
                static_cast<std::uint16_t>(crcLow_ | (static_cast<std::uint16_t>(byte) << 8));
            const std::uint16_t computed =
                crc16(std::span<const std::uint8_t>(checked.data(), frame_.payloadSize + 2U));
            if (received != computed) {
                return {.frame = std::nullopt, .error = ErrorCode::crcMismatch};
            }
            return {.frame = frame_, .error = std::nullopt};
        }
    }
    return {};
}

}  // namespace pc_protocol
