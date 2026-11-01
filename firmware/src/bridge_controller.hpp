#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "motor_protocol.hpp"

namespace bridge {

// Tuning knobs. The motor's own limit is 210 rpm; raise this after bench verification.
inline constexpr std::int16_t MAX_VELOCITY_RPM = 30;
inline constexpr std::uint32_t DISABLED_REFRESH_PERIOD_MS = 100;
inline constexpr std::uint32_t COMMAND_TIMEOUT_MS = 200;
inline constexpr std::uint32_t FEEDBACK_TIMEOUT_MS = 100;
inline constexpr std::uint32_t ARMING_TIMEOUT_MS = 500;
inline constexpr std::uint32_t STOPPING_DURATION_MS = 500;
inline constexpr std::uint32_t VELOCITY_SEND_PERIOD_MS = 20;
inline constexpr std::uint8_t FEEDBACK_INTERVAL_MS = 10;

enum class State : std::uint8_t { disabled = 0, arming = 1, active = 2, stopping = 3 };

enum class StopReason : std::uint8_t {
    none = 0,
    disableRequested = 1,
    commandTimeout = 2,
    feedbackLost = 3,
    motorFault = 4,
    armingTimeout = 5,
    canTransmitFailed = 6,
    modeLost = 7,
};

enum class CommandResult { accepted, velocityOutOfRange, invalidState };

// Frames produced by one call, in the order they must reach the bus.
struct Output {
    std::array<motor_protocol::CanFrame, 3> frames{};
    std::size_t count = 0;

    void add(const motor_protocol::CanFrame& frame) { frames[count++] = frame; }
};

class Controller {
public:
    [[nodiscard]] CommandResult enable(std::uint32_t nowMs, Output& output);
    [[nodiscard]] CommandResult disable(std::uint32_t nowMs);
    [[nodiscard]] CommandResult setVelocity(std::int16_t firstRpm, std::int16_t secondRpm,
                                            std::uint32_t nowMs);
    void onFeedback(const motor_protocol::Feedback& feedback, std::uint32_t nowMs);
    void onCanTransmitFailure(std::uint32_t nowMs);
    [[nodiscard]] Output tick(std::uint32_t nowMs);

    [[nodiscard]] State state() const { return state_; }
    [[nodiscard]] StopReason stopReason() const { return stopReason_; }

private:
    struct MotorStatus {
        std::uint32_t lastFeedbackMs = 0;
        std::uint8_t mode = 0;
        std::uint8_t fault = 0;
    };

    void beginStopping(StopReason reason, std::uint32_t nowMs);
    void tickActive(std::uint32_t nowMs);
    [[nodiscard]] bool isAnyMotorFaulted() const;
    [[nodiscard]] bool isFeedbackLost(std::uint32_t nowMs) const;
    [[nodiscard]] bool areAllMotorsInVelocityMode() const;
    void enterDisabled(std::uint32_t nowMs, Output& output);

    State state_ = State::disabled;
    StopReason stopReason_ = StopReason::none;
    std::array<MotorStatus, motor_protocol::MOTOR_COUNT> motors_{};
    std::int16_t firstRpm_ = 0;
    std::int16_t secondRpm_ = 0;
    std::uint32_t lastCommandMs_ = 0;
    std::uint32_t stateEnteredMs_ = 0;
    std::uint32_t lastVelocitySendMs_ = 0;
    // Wraps below zero so that the first tick after boot sends immediately.
    std::uint32_t lastDisabledSendMs_ = 0U - DISABLED_REFRESH_PERIOD_MS;
};

}  // namespace bridge
