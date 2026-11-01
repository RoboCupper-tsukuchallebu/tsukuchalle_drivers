#include "motor_protocol.hpp"

namespace motor_protocol {
namespace {

std::uint16_t readBigEndian(const std::array<std::uint8_t, 8>& data, std::size_t offset) {
    return static_cast<std::uint16_t>((static_cast<std::uint16_t>(data[offset]) << 8) |
                                      data[offset + 1]);
}

CanFrame makePerMotorSetting(std::uint16_t id, std::uint8_t value) {
    CanFrame frame{.id = id, .data = {}};
    for (std::uint8_t index = 0; index < MOTOR_COUNT; ++index) {
        frame.data[index] = value;
    }
    return frame;
}

}  // namespace

CanFrame makeVelocityCommand(std::int16_t firstRpm, std::int16_t secondRpm) {
    const auto first = static_cast<std::uint16_t>(firstRpm);
    const auto second = static_cast<std::uint16_t>(secondRpm);
    CanFrame frame{.id = VELOCITY_COMMAND_ID, .data = {}};
    frame.data[0] = static_cast<std::uint8_t>(first >> 8);
    frame.data[1] = static_cast<std::uint8_t>(first & 0xFF);
    frame.data[2] = static_cast<std::uint8_t>(second >> 8);
    frame.data[3] = static_cast<std::uint8_t>(second & 0xFF);
    return frame;
}

CanFrame makeSetMode(std::uint8_t mode) {
    return makePerMotorSetting(SET_MODE_ID, mode);
}

CanFrame makeSetReporting(std::uint8_t intervalMilliseconds) {
    return makePerMotorSetting(SET_REPORTING_ID, intervalMilliseconds);
}

std::optional<Feedback> parseFeedback(const CanFrame& frame) {
    if (frame.id <= FEEDBACK_ID_BASE || frame.id > FEEDBACK_ID_BASE + MOTOR_COUNT) {
        return std::nullopt;
    }
    return Feedback{
        .motorId = static_cast<std::uint8_t>(frame.id - FEEDBACK_ID_BASE),
        .velocityRpm = static_cast<std::int16_t>(readBigEndian(frame.data, 0)),
        .currentRaw = static_cast<std::int16_t>(readBigEndian(frame.data, 2)),
        .angleRaw = readBigEndian(frame.data, 4),
        .fault = frame.data[6],
        .mode = frame.data[7],
    };
}

}  // namespace motor_protocol
