#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace motor_protocol {

inline constexpr std::uint16_t VELOCITY_COMMAND_ID = 0x32;
inline constexpr std::uint16_t SET_MODE_ID = 0x105;
inline constexpr std::uint16_t SET_REPORTING_ID = 0x106;
inline constexpr std::uint16_t FEEDBACK_ID_BASE = 0x96;

inline constexpr std::uint8_t MODE_VELOCITY = 0x02;
inline constexpr std::uint8_t MODE_DISABLE = 0x09;
inline constexpr std::uint8_t MODE_ENABLE = 0x0A;

inline constexpr std::uint8_t MOTOR_COUNT = 2;

struct CanFrame {
    std::uint16_t id = 0;
    std::array<std::uint8_t, 8> data{};
};

struct Feedback {
    std::uint8_t motorId = 0;
    std::int16_t velocityRpm = 0;
    std::int16_t currentRaw = 0;
    std::uint16_t angleRaw = 0;
    std::uint8_t fault = 0;
    std::uint8_t mode = 0;
};

[[nodiscard]] CanFrame makeVelocityCommand(std::int16_t firstRpm, std::int16_t secondRpm);
[[nodiscard]] CanFrame makeSetMode(std::uint8_t mode);
[[nodiscard]] CanFrame makeSetReporting(std::uint8_t intervalMilliseconds);

// Returns nullopt for frames that are not feedback from motors 1..MOTOR_COUNT.
[[nodiscard]] std::optional<Feedback> parseFeedback(const CanFrame& frame);

}  // namespace motor_protocol
