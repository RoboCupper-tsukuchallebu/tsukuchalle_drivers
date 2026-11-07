// VelodyneScan (HDL-32E) -> organized PointCloud2, without PCL.
// The per-point arithmetic is ported from ros-drivers/velodyne velodyne_pointcloud rawdata.cpp
// (RawData::unpack, HDL-32 path); see hdl32e_converter.cpp for the licence.
// tsukuchalle_pointcloud has a similar reader (PointCloud2View) but is deliberately not shared.
#ifndef TSUKUCHALLE_VELODYNE__HDL32E_CONVERTER_HPP_
#define TSUKUCHALLE_VELODYNE__HDL32E_CONVERTER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include "tsukuchalle_velodyne/hdl32e_calibration.hpp"

namespace tsukuchalle_velodyne
{

struct ConversionResult final
{
  sensor_msgs::msg::PointCloud2::UniquePtr cloud;
  // Blocks whose header is not 0xEEFF or whose azimuth is 36000 or more; their 32 cells are NaN.
  std::size_t invalid_block_count;
};

class Hdl32eConverter final
{
public:
  // min_range / max_range stay double: the reference compares a float distance with a double.
  Hdl32eConverter(Hdl32eCalibration calibration, double min_range, double max_range);

  // width 32 (column = ring), height = packet count x 12, point_step 22, is_dense false.
  [[nodiscard]] ConversionResult convert(const velodyne_msgs::msg::VelodyneScan & scan) const;

private:
  // Writes the 32 cells of one block into `row`. Returns false (and writes a NaN row) for an
  // invalid block header or azimuth.
  bool convertBlock(
    const std::uint8_t * block_bytes, int block_index, float packet_time_offset_seconds,
    std::uint8_t * row) const;

  Hdl32eCalibration calibration_;
  double min_range_;
  double max_range_;
  std::vector<float> cos_azimuth_table_;
  std::vector<float> sin_azimuth_table_;
  std::array<std::array<float, LASER_COUNT>, BLOCKS_PER_PACKET> timing_offsets_;
};

}  // namespace tsukuchalle_velodyne

#endif  // TSUKUCHALLE_VELODYNE__HDL32E_CONVERTER_HPP_
