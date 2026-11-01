#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace pc_protocol {

inline constexpr std::uint8_t HEADER_FIRST = 0xAA;
inline constexpr std::uint8_t HEADER_SECOND = 0x55;
inline constexpr std::size_t MAX_PAYLOAD_SIZE = 16;
inline constexpr std::size_t FRAME_OVERHEAD_SIZE = 6;
inline constexpr std::size_t MAX_FRAME_SIZE = MAX_PAYLOAD_SIZE + FRAME_OVERHEAD_SIZE;

enum class MessageType : std::uint8_t {
    enable = 0x01,
    disable = 0x02,
    setVelocity = 0x03,
    feedback = 0x81,
    status = 0x82,
    error = 0x83,
};

enum class ErrorCode : std::uint8_t {
    crcMismatch = 1,
    invalidLength = 2,
    velocityOutOfRange = 3,
    unknownType = 4,
    invalidState = 5,
    canTransmitTimeout = 6,
};

struct Frame {
    std::uint8_t type = 0;
    std::uint8_t payloadSize = 0;
    std::array<std::uint8_t, MAX_PAYLOAD_SIZE> payload{};
};

struct EncodedFrame {
    std::array<std::uint8_t, MAX_FRAME_SIZE> bytes{};
    std::size_t size = 0;
};

struct ParseResult {
    std::optional<Frame> frame;
    std::optional<ErrorCode> error;
};

[[nodiscard]] std::uint16_t crc16(std::span<const std::uint8_t> bytes);

// Precondition: payload.size() <= MAX_PAYLOAD_SIZE.
[[nodiscard]] EncodedFrame encode(MessageType type, std::span<const std::uint8_t> payload);

// Resynchronises on the header after any malformed input.
class StreamParser {
public:
    [[nodiscard]] ParseResult push(std::uint8_t byte);

private:
    enum class Step { headerFirst, headerSecond, type, length, payload, crcLow, crcHigh };

    Step step_ = Step::headerFirst;
    Frame frame_{};
    std::size_t payloadIndex_ = 0;
    std::uint8_t crcLow_ = 0;
};

}  // namespace pc_protocol
