// The arithmetic of convertReturn() is a port of RawData::unpack() (HDL-32 path) in
// ros-drivers/velodyne velodyne_pointcloud/src/lib/rawdata.cpp, and the tables follow
// RawData::buildTimings() / setupSinCosCache(). Expressions, their order and the float/double
// types are kept on purpose so that the output matches the reference.
//
// Original licence:
// Copyright 2007, 2009, 2010, 2012, 2019 Austin Robot Technology, Patrick Beeson, Jack O'Quin,
// Kaarta Inc, Shawn Hanna, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include "tsukuchalle_velodyne/hdl32e_converter.hpp"

#include <cmath>
#include <cstring>
#include <memory>
#include <utility>

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "packet fields are read with memcpy and assume a little-endian host"
#endif

namespace tsukuchalle_velodyne
{
namespace
{

constexpr std::size_t POINT_STEP_BYTES = 22;
constexpr std::size_t X_OFFSET = 0;
constexpr std::size_t Y_OFFSET = 4;
constexpr std::size_t Z_OFFSET = 8;
constexpr std::size_t INTENSITY_OFFSET = 12;
constexpr std::size_t RING_OFFSET = 16;
constexpr std::size_t TIME_OFFSET = 18;
constexpr std::size_t ROW_STEP_BYTES = POINT_STEP_BYTES * LASER_COUNT;

constexpr std::size_t HALF_OF_LASERS_PER_FIRING = 2;  // lasers fire in pairs: index / 2

inline float square(const float value)
{
  return value * value;
}

struct ConvertedPoint final
{
  float x;
  float y;
  float z;
  float intensity;
  float distance;  // after dist_correction, before the two point correction (used for the range test)
};

// raw->blocks[i].data[k + 2] is the intensity byte; `distance` is what the reference passes to addPoint.
float calculateIntensity(
  const LaserCorrection & laser_correction, const std::uint8_t intensity_raw, const float distance)
{
  float min_intensity = laser_correction.min_intensity;
  float max_intensity = laser_correction.max_intensity;

  float intensity = intensity_raw;

  float focal_offset = 256.0f *
    (1.0f - laser_correction.focal_distance / 13100.0f) *
    (1.0f - laser_correction.focal_distance / 13100.0f);
  float focal_slope = laser_correction.focal_slope;
  intensity += focal_slope *
    (std::abs(focal_offset - 256.0f * square((1.0f - distance) / 65535.0f)));
  intensity = (intensity < min_intensity) ? min_intensity : intensity;
  intensity = (intensity > max_intensity) ? max_intensity : intensity;
  return intensity;
}

ConvertedPoint convertReturn(
  const LaserCorrection & laser_correction, const float distance_resolution_m,
  const std::uint16_t distance_raw, const std::uint8_t intensity_raw,
  const float cos_azimuth, const float sin_azimuth)
{
  float distance = distance_raw * distance_resolution_m;
  distance += laser_correction.dist_correction;

  float cos_vert_angle = laser_correction.cos_vert_correction;
  float sin_vert_angle = laser_correction.sin_vert_correction;
  float cos_rot_correction = laser_correction.cos_rot_correction;
  float sin_rot_correction = laser_correction.sin_rot_correction;

  // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
  // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
  float cos_rot_angle = cos_azimuth * cos_rot_correction + sin_azimuth * sin_rot_correction;
  float sin_rot_angle = sin_azimuth * cos_rot_correction - cos_azimuth * sin_rot_correction;

  float horiz_offset = laser_correction.horiz_offset_correction;
  float vert_offset = laser_correction.vert_offset_correction;

  float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

  float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
  float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

  if (xx < 0.0f) {
    xx = -xx;
  }
  if (yy < 0.0f) {
    yy = -yy;
  }

  float distance_corr_x = 0;
  float distance_corr_y = 0;
  if (laser_correction.two_pt_correction_available) {
    distance_corr_x =
      (laser_correction.dist_correction - laser_correction.dist_correction_x) *
      (xx - 2.4f) / (25.04f - 2.4f) +
      laser_correction.dist_correction_x;
    distance_corr_x -= laser_correction.dist_correction;
    distance_corr_y =
      (laser_correction.dist_correction - laser_correction.dist_correction_y) *
      (yy - 1.93f) / (25.04f - 1.93f) +
      laser_correction.dist_correction_y;
    distance_corr_y -= laser_correction.dist_correction;
  }

  float distance_x = distance + distance_corr_x;
  xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
  float x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

  float distance_y = distance + distance_corr_y;
  xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
  float y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

  // Using distance_y is not symmetric, but the velodyne manual does this.
  float z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

  // Standard ROS coordinate system (right-hand rule).
  return ConvertedPoint{y, -x, z, calculateIntensity(laser_correction, intensity_raw, distance),
    distance};
}

void writeCell(
  std::uint8_t * row, const int column, const float x, const float y, const float z,
  const float intensity, const std::uint16_t ring, const float time)
{
  std::uint8_t * cell = row + static_cast<std::size_t>(column) * POINT_STEP_BYTES;
  std::memcpy(cell + X_OFFSET, &x, sizeof(float));
  std::memcpy(cell + Y_OFFSET, &y, sizeof(float));
  std::memcpy(cell + Z_OFFSET, &z, sizeof(float));
  std::memcpy(cell + INTENSITY_OFFSET, &intensity, sizeof(float));
  std::memcpy(cell + RING_OFFSET, &ring, sizeof(std::uint16_t));
  std::memcpy(cell + TIME_OFFSET, &time, sizeof(float));
}

void writeNanCell(
  std::uint8_t * row, const int ring, const float time)
{
  const float not_a_number = ::nanf("");
  writeCell(
    row, ring, not_a_number, not_a_number, not_a_number, not_a_number,
    static_cast<std::uint16_t>(ring), time);
}

void initializeCloudLayout(sensor_msgs::msg::PointCloud2 & cloud)
{
  using sensor_msgs::msg::PointField;
  const struct
  {
    const char * name;
    std::uint32_t offset;
    std::uint8_t datatype;
  } fields[] = {
    {"x", X_OFFSET, PointField::FLOAT32}, {"y", Y_OFFSET, PointField::FLOAT32},
    {"z", Z_OFFSET, PointField::FLOAT32}, {"intensity", INTENSITY_OFFSET, PointField::FLOAT32},
    {"ring", RING_OFFSET, PointField::UINT16}, {"time", TIME_OFFSET, PointField::FLOAT32}};
  for (const auto & field_description : fields) {
    PointField field;
    field.name = field_description.name;
    field.offset = field_description.offset;
    field.datatype = field_description.datatype;
    field.count = 1;
    cloud.fields.push_back(field);
  }
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  cloud.point_step = POINT_STEP_BYTES;
  cloud.width = LASER_COUNT;
  cloud.row_step = ROW_STEP_BYTES;
}

}  // namespace

Hdl32eConverter::Hdl32eConverter(
  Hdl32eCalibration calibration, const double min_range, const double max_range)
: calibration_(std::move(calibration)),
  min_range_(min_range),
  max_range_(max_range),
  cos_azimuth_table_(AZIMUTH_UNITS_PER_REVOLUTION),
  sin_azimuth_table_(AZIMUTH_UNITS_PER_REVOLUTION),
  timing_offsets_{}
{
  // angles::from_degrees(degrees) is degrees * M_PI / 180.0 on a double.
  for (std::uint16_t rot_index = 0; rot_index < AZIMUTH_UNITS_PER_REVOLUTION; ++rot_index) {
    const double degrees = AZIMUTH_RESOLUTION_DEGREES * rot_index;
    float rotation = degrees * M_PI / 180.0;
    cos_azimuth_table_[rot_index] = ::cosf(rotation);
    sin_azimuth_table_[rot_index] = ::sinf(rotation);
  }
  for (std::size_t block_index = 0; block_index < timing_offsets_.size(); ++block_index) {
    for (std::size_t laser_index = 0; laser_index < timing_offsets_[block_index].size();
      ++laser_index)
    {
      double data_block_index = block_index;
      double data_point_index = laser_index / HALF_OF_LASERS_PER_FIRING;
      timing_offsets_[block_index][laser_index] =
        (FULL_FIRING_CYCLE_SECONDS * data_block_index) + (SINGLE_FIRING_SECONDS * data_point_index);
    }
  }
}

bool Hdl32eConverter::convertBlock(
  const std::uint8_t * block_bytes, const int block_index, const float packet_time_offset_seconds,
  std::uint8_t * row) const
{
  std::uint16_t header = 0;
  std::uint16_t azimuth_hundredths_of_degree = 0;
  std::memcpy(&header, block_bytes, sizeof(header));
  std::memcpy(&azimuth_hundredths_of_degree, block_bytes + 2, sizeof(azimuth_hundredths_of_degree));
  const bool is_valid = header == UPPER_BANK_HEADER &&
    azimuth_hundredths_of_degree < AZIMUTH_UNITS_PER_REVOLUTION;

  for (int laser_index = 0; laser_index < LASER_COUNT; ++laser_index) {
    const LaserCorrection & laser_correction = calibration_.laser_corrections()[laser_index];
    const float time = timing_offsets_[block_index][laser_index] + packet_time_offset_seconds;
    if (!is_valid) {
      writeNanCell(row, laser_correction.laser_ring, time);
      continue;
    }
    const std::uint8_t * laser_bytes =
      block_bytes + BLOCK_HEADER_BYTES + laser_index * LASER_RECORD_BYTES;
    std::uint16_t distance_raw = 0;
    std::memcpy(&distance_raw, laser_bytes, sizeof(distance_raw));
    if (distance_raw == 0) {  // no valid laser beam return
      writeNanCell(row, laser_correction.laser_ring, time);
      continue;
    }
    const ConvertedPoint point = convertReturn(
      laser_correction, calibration_.distance_resolution_m(), distance_raw, laser_bytes[2],
      cos_azimuth_table_[azimuth_hundredths_of_degree],
      sin_azimuth_table_[azimuth_hundredths_of_degree]);
    if (point.distance >= min_range_ && point.distance <= max_range_) {
      writeCell(
        row, laser_correction.laser_ring, point.x, point.y, point.z, point.intensity,
        static_cast<std::uint16_t>(laser_correction.laser_ring), time);
    } else {
      writeNanCell(row, laser_correction.laser_ring, time);
    }
  }
  return is_valid;
}

ConversionResult Hdl32eConverter::convert(const velodyne_msgs::msg::VelodyneScan & scan) const
{
  auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
  initializeCloudLayout(*cloud);
  cloud->header = scan.header;
  cloud->height = static_cast<std::uint32_t>(scan.packets.size()) * BLOCKS_PER_PACKET;
  cloud->data.resize(ROW_STEP_BYTES * cloud->height);

  std::size_t invalid_block_count = 0;
  std::size_t row_index = 0;
  const rclcpp::Time scan_start_time(scan.header.stamp);
  for (const velodyne_msgs::msg::VelodynePacket & packet : scan.packets) {
    float time_diff_start_to_this_packet = (rclcpp::Time(packet.stamp) - scan_start_time).seconds();
    for (int block_index = 0; block_index < BLOCKS_PER_PACKET; ++block_index, ++row_index) {
      const bool is_valid = convertBlock(
        packet.data.data() + static_cast<std::size_t>(block_index) * BLOCK_SIZE_BYTES, block_index,
        time_diff_start_to_this_packet, cloud->data.data() + row_index * ROW_STEP_BYTES);
      invalid_block_count += is_valid ? 0 : 1;
    }
  }
  return ConversionResult{std::move(cloud), invalid_block_count};
}

}  // namespace tsukuchalle_velodyne
