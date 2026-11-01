#pragma once

#include <cstdint>
#include <optional>

#include "motor_protocol.hpp"

// Minimal driver for the XL2515 (MCP2515 clone) on the Waveshare RP2350-CAN.
// Fixed at 500 kbit/s from the board's 16 MHz crystal, standard frames, 8-byte payloads.
namespace xl2515 {

[[nodiscard]] bool initialize();
[[nodiscard]] bool isTransmitBusy();
void abortTransmit();
void transmit(const motor_protocol::CanFrame& frame);
[[nodiscard]] std::optional<motor_protocol::CanFrame> receive();

}  // namespace xl2515
