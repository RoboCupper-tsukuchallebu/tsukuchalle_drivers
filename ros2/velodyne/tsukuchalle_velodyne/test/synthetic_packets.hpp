// Synthetic HDL-32E packets for the tests. Same layout as ros2/velodyne/hdl32e_packets.py:
// 1206 B, little-endian, header 0xEEFF, azimuth 0..35999 spread over 181 packets x 12 firings.
#ifndef TSUKUCHALLE_VELODYNE__TEST__SYNTHETIC_PACKETS_HPP_
#define TSUKUCHALLE_VELODYNE__TEST__SYNTHETIC_PACKETS_HPP_

#include <cstddef>
#include <cstdint>
#include <random>

#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include "tsukuchalle_velodyne/hdl32e_packet.hpp"

namespace tsukuchalle_velodyne::test
{

constexpr int PACKETS_PER_REVOLUTION = 181;
constexpr double DISTANCE_UNIT_METERS = 0.002;
constexpr std::int32_t PACKET_INTERVAL_NANOSECONDS = 553000;
constexpr std::int32_t SCAN_STAMP_SECONDS = 1000;

inline std::size_t blockOffset(const int block_index)
{
  return static_cast<std::size_t>(block_index) * BLOCK_SIZE_BYTES;
}

inline std::size_t laserOffset(const int block_index, const int laser_index)
{
  return blockOffset(block_index) + BLOCK_HEADER_BYTES +
         static_cast<std::size_t>(laser_index) * LASER_RECORD_BYTES;
}

inline void writeUint16(
  velodyne_msgs::msg::VelodynePacket & packet, const std::size_t offset, const std::uint16_t value)
{
  packet.data[offset] = static_cast<std::uint8_t>(value & 0xFF);
  packet.data[offset + 1] = static_cast<std::uint8_t>(value >> 8);
}

inline void setBlockHeader(
  velodyne_msgs::msg::VelodynePacket & packet, const int block_index, const std::uint16_t header)
{
  writeUint16(packet, blockOffset(block_index), header);
}

inline void setBlockAzimuth(
  velodyne_msgs::msg::VelodynePacket & packet, const int block_index,
  const std::uint16_t azimuth_hundredths_of_degree)
{
  writeUint16(packet, blockOffset(block_index) + 2, azimuth_hundredths_of_degree);
}

inline void setLaser(
  velodyne_msgs::msg::VelodynePacket & packet, const int block_index, const int laser_index,
  const std::uint16_t distance_raw, const std::uint8_t intensity)
{
  const std::size_t offset = laserOffset(block_index, laser_index);
  writeUint16(packet, offset, distance_raw);
  packet.data[offset + 2] = intensity;
}

inline velodyne_msgs::msg::VelodynePacket makeBlankPacket(
  const int packet_index, const int packet_count)
{
  velodyne_msgs::msg::VelodynePacket packet;
  packet.data.fill(0);
  for (int block_index = 0; block_index < BLOCKS_PER_PACKET; ++block_index) {
    const int firing_index = packet_index * BLOCKS_PER_PACKET + block_index;
    const int firings_per_revolution = PACKETS_PER_REVOLUTION * BLOCKS_PER_PACKET;
    setBlockHeader(packet, block_index, UPPER_BANK_HEADER);
    setBlockAzimuth(
      packet, block_index,
      static_cast<std::uint16_t>(
        (firing_index % firings_per_revolution) * AZIMUTH_UNITS_PER_REVOLUTION /
        firings_per_revolution));
  }
  const std::int64_t packets_before_end = packet_count - 1 - packet_index;
  const std::int64_t offset_nanoseconds = packets_before_end * PACKET_INTERVAL_NANOSECONDS;
  const std::int64_t total_nanoseconds = 1000000000LL - offset_nanoseconds;
  packet.stamp.sec = SCAN_STAMP_SECONDS - 1 + static_cast<std::int32_t>(total_nanoseconds / 1000000000LL);
  packet.stamp.nanosec = static_cast<std::uint32_t>(total_nanoseconds % 1000000000LL);
  return packet;
}

inline velodyne_msgs::msg::VelodyneScan makeScanHeader()
{
  velodyne_msgs::msg::VelodyneScan scan;
  scan.header.frame_id = "velodyne";
  scan.header.stamp.sec = SCAN_STAMP_SECONDS;
  scan.header.stamp.nanosec = 0;
  return scan;
}

// Every channel returns `distance_meters` with `intensity`.
inline velodyne_msgs::msg::VelodyneScan makeUniformScan(
  const int packet_count, const double distance_meters, const std::uint8_t intensity = 100)
{
  velodyne_msgs::msg::VelodyneScan scan = makeScanHeader();
  const auto distance_raw = static_cast<std::uint16_t>(distance_meters / DISTANCE_UNIT_METERS + 0.5);
  for (int packet_index = 0; packet_index < packet_count; ++packet_index) {
    velodyne_msgs::msg::VelodynePacket packet = makeBlankPacket(packet_index, packet_count);
    for (int block_index = 0; block_index < BLOCKS_PER_PACKET; ++block_index) {
      for (int laser_index = 0; laser_index < LASER_COUNT; ++laser_index) {
        setLaser(packet, block_index, laser_index, distance_raw, intensity);
      }
    }
    scan.packets.push_back(packet);
  }
  return scan;
}

// Fixed-seed mix: 30 % no return, 10 % under 0.9 m, 55 % in range, 5 % over 130 m; intensity 0..255.
inline velodyne_msgs::msg::VelodyneScan makeRandomScan(
  const int packet_count, const std::uint32_t seed)
{
  constexpr std::uint16_t LAST_RAW_UNDER_MIN_RANGE = 449;    // 0.898 m
  constexpr std::uint16_t FIRST_RAW_OVER_MAX_RANGE = 65100;  // 130.2 m
  constexpr std::uint16_t LAST_RAW_IN_RANGE = 60000;         // 120 m
  std::mt19937 generator(seed);
  std::uniform_int_distribution<int> percent(0, 99);
  std::uniform_int_distribution<int> intensity_values(0, 255);
  std::uniform_int_distribution<int> under_range(1, LAST_RAW_UNDER_MIN_RANGE);
  std::uniform_int_distribution<int> in_range(LAST_RAW_UNDER_MIN_RANGE + 2, LAST_RAW_IN_RANGE);
  std::uniform_int_distribution<int> over_range(FIRST_RAW_OVER_MAX_RANGE, 65535);

  velodyne_msgs::msg::VelodyneScan scan = makeScanHeader();
  for (int packet_index = 0; packet_index < packet_count; ++packet_index) {
    velodyne_msgs::msg::VelodynePacket packet = makeBlankPacket(packet_index, packet_count);
    for (int block_index = 0; block_index < BLOCKS_PER_PACKET; ++block_index) {
      for (int laser_index = 0; laser_index < LASER_COUNT; ++laser_index) {
        const int draw = percent(generator);
        int distance_raw = 0;
        if (draw >= 30 && draw < 40) {
          distance_raw = under_range(generator);
        } else if (draw >= 40 && draw < 95) {
          distance_raw = in_range(generator);
        } else if (draw >= 95) {
          distance_raw = over_range(generator);
        }
        setLaser(
          packet, block_index, laser_index, static_cast<std::uint16_t>(distance_raw),
          static_cast<std::uint8_t>(intensity_values(generator)));
      }
    }
    scan.packets.push_back(packet);
  }
  return scan;
}

}  // namespace tsukuchalle_velodyne::test

#endif  // TSUKUCHALLE_VELODYNE__TEST__SYNTHETIC_PACKETS_HPP_
