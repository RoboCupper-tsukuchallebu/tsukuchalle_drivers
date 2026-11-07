// Velodyne HDL-32E data packet constants (1206 B, little-endian).
// Layout: 12 blocks of (header 0xEEFF, azimuth in 1/100 degree, 32 x (distance u16, intensity u8)),
// then a 4 B timestamp and 2 factory bytes. Another model would replace this header.
#ifndef TSUKUCHALLE_VELODYNE__HDL32E_PACKET_HPP_
#define TSUKUCHALLE_VELODYNE__HDL32E_PACKET_HPP_

#include <cstddef>
#include <cstdint>

namespace tsukuchalle_velodyne
{

constexpr int LASER_COUNT = 32;
constexpr int BLOCKS_PER_PACKET = 12;
constexpr std::size_t PACKET_SIZE_BYTES = 1206;

constexpr std::size_t BLOCK_HEADER_BYTES = 4;
constexpr std::size_t LASER_RECORD_BYTES = 3;
constexpr std::size_t BLOCK_SIZE_BYTES = BLOCK_HEADER_BYTES + LASER_COUNT * LASER_RECORD_BYTES;
static_assert(BLOCK_SIZE_BYTES == 100, "an HDL-32E block is 100 bytes");

constexpr std::uint16_t UPPER_BANK_HEADER = 0xEEFF;
constexpr std::uint16_t AZIMUTH_UNITS_PER_REVOLUTION = 36000;
constexpr float AZIMUTH_RESOLUTION_DEGREES = 0.01f;

// Seconds from the first firing of a packet to a block, and between laser pairs inside a block.
constexpr double FULL_FIRING_CYCLE_SECONDS = 46.080 * 1e-6;
constexpr double SINGLE_FIRING_SECONDS = 1.152 * 1e-6;

}  // namespace tsukuchalle_velodyne

#endif  // TSUKUCHALLE_VELODYNE__HDL32E_PACKET_HPP_
