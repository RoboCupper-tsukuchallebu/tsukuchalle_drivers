#include "bridge_controller.hpp"

#include <algorithm>

namespace bridge {
namespace {

using motor_protocol::makeSetMode;
using motor_protocol::makeSetReporting;
using motor_protocol::makeVelocityCommand;

bool isVelocityInRange(std::int16_t rpm) {
    return rpm >= -MAX_VELOCITY_RPM && rpm <= MAX_VELOCITY_RPM;
}

// Unsigned subtraction stays correct across the 32-bit millisecond wrap.
bool hasElapsed(std::uint32_t nowMs, std::uint32_t sinceMs, std::uint32_t durationMs) {
    return nowMs - sinceMs >= durationMs;
}

}  // namespace

CommandResult Controller::enable(std::uint32_t nowMs, Output& output) {
    if (state_ != State::disabled) {
        return CommandResult::invalidState;
    }
    if (isAnyMotorFaulted()) {
        return CommandResult::invalidState;
    }
    firstRpm_ = 0;
    secondRpm_ = 0;
    // Require fresh confirmation: a stale velocity-mode report must not arm the bridge.
    std::for_each(motors_.begin(), motors_.end(), [](MotorStatus& motor) { motor.mode = 0; });
    state_ = State::arming;
    stopReason_ = StopReason::none;
    stateEnteredMs_ = nowMs;
    lastVelocitySendMs_ = nowMs;
    // Clear any target the motors retained before they are enabled.
    output.add(makeVelocityCommand(0, 0));
    output.add(makeSetMode(motor_protocol::MODE_ENABLE));
    output.add(makeSetMode(motor_protocol::MODE_VELOCITY));
    return CommandResult::accepted;
}

CommandResult Controller::disable(std::uint32_t nowMs) {
    if (state_ == State::arming || state_ == State::active) {
        beginStopping(StopReason::disableRequested, nowMs);
    }
    return CommandResult::accepted;
}

CommandResult Controller::setVelocity(std::int16_t firstRpm, std::int16_t secondRpm,
                                      std::uint32_t nowMs) {
    // A rejected command deliberately does not feed the timeout.
    if (!isVelocityInRange(firstRpm) || !isVelocityInRange(secondRpm)) {
        return CommandResult::velocityOutOfRange;
    }
    if (state_ != State::active) {
        return CommandResult::invalidState;
    }
    firstRpm_ = firstRpm;
    secondRpm_ = secondRpm;
    lastCommandMs_ = nowMs;
    return CommandResult::accepted;
}

void Controller::onFeedback(const motor_protocol::Feedback& feedback, std::uint32_t nowMs) {
    if (feedback.motorId < 1 || feedback.motorId > motors_.size()) {
        return;
    }
    MotorStatus& motor = motors_[feedback.motorId - 1U];
    motor.lastFeedbackMs = nowMs;
    motor.mode = feedback.mode;
    motor.fault = feedback.fault;
}

void Controller::onCanTransmitFailure(std::uint32_t nowMs) {
    if (state_ == State::arming || state_ == State::active) {
        beginStopping(StopReason::canTransmitFailed, nowMs);
    }
}

void Controller::enterDisabled(std::uint32_t nowMs, Output& output) {
    state_ = State::disabled;
    lastDisabledSendMs_ = nowMs;
    output.add(makeVelocityCommand(0, 0));
    output.add(makeSetMode(motor_protocol::MODE_DISABLE));
    output.add(makeSetReporting(FEEDBACK_INTERVAL_MS));
}

Output Controller::tick(std::uint32_t nowMs) {
    Output output;
    switch (state_) {
        case State::disabled:
            // Repeated forever: a motor may power up or reconnect at any time and
            // would otherwise keep its last target or its power-on defaults.
            if (hasElapsed(nowMs, lastDisabledSendMs_, DISABLED_REFRESH_PERIOD_MS)) {
                enterDisabled(nowMs, output);
            }
            return output;
        case State::arming:
            if (isAnyMotorFaulted() || hasElapsed(nowMs, stateEnteredMs_, ARMING_TIMEOUT_MS)) {
                stopReason_ =
                    isAnyMotorFaulted() ? StopReason::motorFault : StopReason::armingTimeout;
                enterDisabled(nowMs, output);
                return output;
            }
            if (areAllMotorsInVelocityMode()) {
                state_ = State::active;
                stateEnteredMs_ = nowMs;
                lastCommandMs_ = nowMs;
            }
            break;
        case State::active:
            tickActive(nowMs);
            break;
        case State::stopping:
            if (hasElapsed(nowMs, stateEnteredMs_, STOPPING_DURATION_MS)) {
                enterDisabled(nowMs, output);
                return output;
            }
            break;
    }
    if (hasElapsed(nowMs, lastVelocitySendMs_, VELOCITY_SEND_PERIOD_MS)) {
        lastVelocitySendMs_ = nowMs;
        output.add(makeVelocityCommand(firstRpm_, secondRpm_));
    }
    return output;
}

void Controller::tickActive(std::uint32_t nowMs) {
    if (isAnyMotorFaulted()) {
        beginStopping(StopReason::motorFault, nowMs);
    } else if (isFeedbackLost(nowMs)) {
        beginStopping(StopReason::feedbackLost, nowMs);
    } else if (!areAllMotorsInVelocityMode()) {
        beginStopping(StopReason::modeLost, nowMs);
    } else if (hasElapsed(nowMs, lastCommandMs_, COMMAND_TIMEOUT_MS)) {
        beginStopping(StopReason::commandTimeout, nowMs);
    }
}

void Controller::beginStopping(StopReason reason, std::uint32_t nowMs) {
    state_ = State::stopping;
    stopReason_ = reason;
    stateEnteredMs_ = nowMs;
    firstRpm_ = 0;
    secondRpm_ = 0;
    lastVelocitySendMs_ = nowMs - VELOCITY_SEND_PERIOD_MS;
}

bool Controller::isAnyMotorFaulted() const {
    return std::any_of(motors_.begin(), motors_.end(),
                       [](const MotorStatus& motor) { return motor.fault != 0; });
}

bool Controller::isFeedbackLost(std::uint32_t nowMs) const {
    return std::any_of(motors_.begin(), motors_.end(), [nowMs](const MotorStatus& motor) {
        return hasElapsed(nowMs, motor.lastFeedbackMs, FEEDBACK_TIMEOUT_MS);
    });
}

bool Controller::areAllMotorsInVelocityMode() const {
    return std::all_of(motors_.begin(), motors_.end(), [](const MotorStatus& motor) {
        return motor.mode == motor_protocol::MODE_VELOCITY;
    });
}

}  // namespace bridge
