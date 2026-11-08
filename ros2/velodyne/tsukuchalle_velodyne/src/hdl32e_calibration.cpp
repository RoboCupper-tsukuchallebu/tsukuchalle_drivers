// Calibration reader. The key names, the float types, the cached cosine/sine values and the
// laser -> ring selection sort follow velodyne_pointcloud/src/lib/calibration.cpp
// (ros-drivers/velodyne, BSD-3-Clause, Copyright 2012, 2019 Austin Robot Technology,
// Piyush Khandelwal, Joshua Whitley). The validation is an addition of this package.
#include "tsukuchalle_velodyne/hdl32e_calibration.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace tsukuchalle_velodyne
{
namespace
{

constexpr char NUM_LASERS_KEY[] = "num_lasers";
constexpr char DISTANCE_RESOLUTION_KEY[] = "distance_resolution";
constexpr char LASERS_KEY[] = "lasers";
constexpr char LASER_ID_KEY[] = "laser_id";
constexpr char ROT_CORRECTION_KEY[] = "rot_correction";
constexpr char VERT_CORRECTION_KEY[] = "vert_correction";
constexpr char DIST_CORRECTION_KEY[] = "dist_correction";
constexpr char TWO_PT_CORRECTION_AVAILABLE_KEY[] = "two_pt_correction_available";
constexpr char DIST_CORRECTION_X_KEY[] = "dist_correction_x";
constexpr char DIST_CORRECTION_Y_KEY[] = "dist_correction_y";
constexpr char VERT_OFFSET_CORRECTION_KEY[] = "vert_offset_correction";
constexpr char HORIZ_OFFSET_CORRECTION_KEY[] = "horiz_offset_correction";
constexpr char MAX_INTENSITY_KEY[] = "max_intensity";
constexpr char MIN_INTENSITY_KEY[] = "min_intensity";
constexpr char FOCAL_DISTANCE_KEY[] = "focal_distance";
constexpr char FOCAL_SLOPE_KEY[] = "focal_slope";

constexpr float DEFAULT_MAX_INTENSITY = 255.0f;
constexpr float DEFAULT_MIN_INTENSITY = 0.0f;

YAML::Node requireKey(const YAML::Node & node, const char * key)
{
  const YAML::Node value = node[key];
  if (!value.IsDefined()) {
    throw std::runtime_error(std::string("calibration: missing key '") + key + "'");
  }
  return value;
}

template<typename Value>
Value readRequired(const YAML::Node & node, const char * key)
{
  return requireKey(node, key).as<Value>();
}

template<typename Value>
Value readOptional(const YAML::Node & node, const char * key, const Value default_value)
{
  const YAML::Node value = node[key];
  return value.IsDefined() ? value.as<Value>() : default_value;
}

LaserCorrection readLaserCorrection(const YAML::Node & laser_node)
{
  LaserCorrection laser_correction{};
  laser_correction.rot_correction = readRequired<float>(laser_node, ROT_CORRECTION_KEY);
  laser_correction.vert_correction = readRequired<float>(laser_node, VERT_CORRECTION_KEY);
  laser_correction.dist_correction = readRequired<float>(laser_node, DIST_CORRECTION_KEY);
  laser_correction.two_pt_correction_available =
    readOptional<bool>(laser_node, TWO_PT_CORRECTION_AVAILABLE_KEY, false);
  laser_correction.dist_correction_x = readRequired<float>(laser_node, DIST_CORRECTION_X_KEY);
  laser_correction.dist_correction_y = readRequired<float>(laser_node, DIST_CORRECTION_Y_KEY);
  laser_correction.vert_offset_correction =
    readRequired<float>(laser_node, VERT_OFFSET_CORRECTION_KEY);
  laser_correction.horiz_offset_correction =
    readOptional<float>(laser_node, HORIZ_OFFSET_CORRECTION_KEY, 0.0f);
  laser_correction.max_intensity = static_cast<int>(
    ::floorf(readOptional<float>(laser_node, MAX_INTENSITY_KEY, DEFAULT_MAX_INTENSITY)));
  laser_correction.min_intensity = static_cast<int>(
    ::floorf(readOptional<float>(laser_node, MIN_INTENSITY_KEY, DEFAULT_MIN_INTENSITY)));
  laser_correction.focal_distance = readRequired<float>(laser_node, FOCAL_DISTANCE_KEY);
  laser_correction.focal_slope = readRequired<float>(laser_node, FOCAL_SLOPE_KEY);

  laser_correction.cos_rot_correction = ::cosf(laser_correction.rot_correction);
  laser_correction.sin_rot_correction = ::sinf(laser_correction.rot_correction);
  laser_correction.cos_vert_correction = ::cosf(laser_correction.vert_correction);
  laser_correction.sin_vert_correction = ::sinf(laser_correction.vert_correction);
  laser_correction.laser_ring = 0;  // assigned after all lasers are read
  return laser_correction;
}

// The reference leaves laser_ring at 0 for the second of two equal angles; this rejects them.
void rejectUnusableVerticalAngles(const std::array<LaserCorrection, LASER_COUNT> & corrections)
{
  for (int laser_index = 0; laser_index < LASER_COUNT; ++laser_index) {
    const float vert_correction = corrections[laser_index].vert_correction;
    if (!std::isfinite(vert_correction)) {
      throw std::runtime_error("calibration: vert_correction is not finite");
    }
    for (int other_index = 0; other_index < laser_index; ++other_index) {
      if (corrections[other_index].vert_correction == vert_correction) {
        throw std::runtime_error("calibration: two lasers have the same vert_correction (as float)");
      }
    }
  }
}

// For each ring, find the next-smallest vertical angle (selection sort, as in the reference).
void assignRings(std::array<LaserCorrection, LASER_COUNT> & corrections)
{
  double next_angle = -std::numeric_limits<double>::infinity();
  for (int ring = 0; ring < LASER_COUNT; ++ring) {
    double smallest_seen = std::numeric_limits<double>::infinity();
    int next_index = LASER_COUNT;
    for (int laser_index = 0; laser_index < LASER_COUNT; ++laser_index) {
      const double angle = corrections[laser_index].vert_correction;
      if (next_angle < angle && angle < smallest_seen) {
        smallest_seen = angle;
        next_index = laser_index;
      }
    }
    if (next_index < LASER_COUNT) {
      corrections[next_index].laser_ring = ring;
      next_angle = smallest_seen;
    }
  }
}

}  // namespace

Hdl32eCalibration Hdl32eCalibration::loadFromYamlText(const std::string & yaml_text)
{
  try {
    const YAML::Node document = YAML::Load(yaml_text);
    if (!document.IsMap()) {
      throw std::runtime_error("calibration: the document is not a mapping");
    }
    if (readRequired<int>(document, NUM_LASERS_KEY) != LASER_COUNT) {
      throw std::runtime_error("calibration: num_lasers must be 32");
    }
    Hdl32eCalibration calibration;
    calibration.distance_resolution_m_ = readRequired<float>(document, DISTANCE_RESOLUTION_KEY);
    if (!(calibration.distance_resolution_m_ > 0.0f) ||
      !std::isfinite(calibration.distance_resolution_m_))
    {
      throw std::runtime_error("calibration: distance_resolution must be positive");
    }

    const YAML::Node lasers = requireKey(document, LASERS_KEY);
    if (!lasers.IsSequence() || lasers.size() != static_cast<std::size_t>(LASER_COUNT)) {
      throw std::runtime_error("calibration: lasers must hold exactly 32 entries");
    }
    std::array<bool, LASER_COUNT> laser_id_seen{};
    for (const YAML::Node & laser_node : lasers) {
      const int laser_id = readRequired<int>(laser_node, LASER_ID_KEY);
      if (laser_id < 0 || laser_id >= LASER_COUNT || laser_id_seen[laser_id]) {
        throw std::runtime_error("calibration: laser_id must cover 0..31 exactly once");
      }
      laser_id_seen[laser_id] = true;
      calibration.laser_corrections_[laser_id] = readLaserCorrection(laser_node);
    }
    rejectUnusableVerticalAngles(calibration.laser_corrections_);
    assignRings(calibration.laser_corrections_);
    return calibration;
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(std::string("calibration: YAML error: ") + error.what());
  }
}

Hdl32eCalibration Hdl32eCalibration::loadFromYamlFile(const std::string & file_path)
{
  std::ifstream file_stream(file_path);
  if (!file_stream.is_open()) {
    throw std::runtime_error("calibration: cannot open '" + file_path + "'");
  }
  std::ostringstream text_stream;
  text_stream << file_stream.rdbuf();
  return loadFromYamlText(text_stream.str());
}

}  // namespace tsukuchalle_velodyne
