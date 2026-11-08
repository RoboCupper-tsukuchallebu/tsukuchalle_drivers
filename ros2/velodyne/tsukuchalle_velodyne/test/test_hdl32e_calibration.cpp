#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "tsukuchalle_velodyne/hdl32e_calibration.hpp"

namespace tsukuchalle_velodyne
{
namespace
{

const std::string STANDARD_CALIBRATION_PATH =
  std::string(TSUKUCHALLE_VELODYNE_CONFIG_DIRECTORY) + "/32db.yaml";

std::string readStandardCalibrationText()
{
  std::ifstream file_stream(STANDARD_CALIBRATION_PATH);
  std::ostringstream text_stream;
  text_stream << file_stream.rdbuf();
  return text_stream.str();
}

std::string replaceFirst(std::string text, const std::string & from, const std::string & to)
{
  const std::size_t position = text.find(from);
  EXPECT_NE(position, std::string::npos) << from;
  return text.replace(position, from.size(), to);
}

std::string replaceAll(std::string text, const std::string & from, const std::string & to)
{
  for (std::size_t position = text.find(from); position != std::string::npos;
    position = text.find(from, position + to.size()))
  {
    text.replace(position, from.size(), to);
  }
  return text;
}

TEST(Hdl32eCalibration, loadsStandardFile)
{
  const Hdl32eCalibration calibration =
    Hdl32eCalibration::loadFromYamlFile(STANDARD_CALIBRATION_PATH);

  EXPECT_FLOAT_EQ(calibration.distance_resolution_m(), 0.002f);
  EXPECT_FLOAT_EQ(calibration.laser_corrections()[0].vert_correction, -0.5352924815866609f);
  EXPECT_FLOAT_EQ(calibration.laser_corrections()[31].vert_correction, 0.18622663118779495f);
  EXPECT_FALSE(calibration.laser_corrections()[0].two_pt_correction_available);
  EXPECT_EQ(calibration.laser_corrections()[0].max_intensity, 255);
  EXPECT_EQ(calibration.laser_corrections()[0].min_intensity, 0);
}

// An implementation that assumes laser id == ring fails at laser 1 (ring 16).
TEST(Hdl32eCalibration, mapsLaserToRingByAscendingVerticalAngle)
{
  const std::array<int, LASER_COUNT> expected_rings = {
    0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23,
    8, 24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31};
  const Hdl32eCalibration calibration =
    Hdl32eCalibration::loadFromYamlFile(STANDARD_CALIBRATION_PATH);

  for (int laser_index = 0; laser_index < LASER_COUNT; ++laser_index) {
    EXPECT_EQ(calibration.laser_corrections()[laser_index].laser_ring, expected_rings[laser_index])
      << "laser " << laser_index;
  }
}

TEST(Hdl32eCalibration, cachesCosineAndSineAsFloat)
{
  const Hdl32eCalibration calibration =
    Hdl32eCalibration::loadFromYamlFile(STANDARD_CALIBRATION_PATH);
  const LaserCorrection & laser_correction = calibration.laser_corrections()[5];

  EXPECT_EQ(laser_correction.cos_vert_correction, ::cosf(laser_correction.vert_correction));
  EXPECT_EQ(laser_correction.sin_vert_correction, ::sinf(laser_correction.vert_correction));
  EXPECT_EQ(laser_correction.cos_rot_correction, 1.0f);
  EXPECT_EQ(laser_correction.sin_rot_correction, 0.0f);
}

TEST(Hdl32eCalibration, appliesDefaultsForOptionalKeys)
{
  const Hdl32eCalibration calibration =
    Hdl32eCalibration::loadFromYamlText(readStandardCalibrationText());

  // 32db.yaml has none of these keys.
  EXPECT_FALSE(calibration.laser_corrections()[3].two_pt_correction_available);
  EXPECT_EQ(calibration.laser_corrections()[3].horiz_offset_correction, 0.0f);
}

TEST(Hdl32eCalibration, rejectsMissingFile)
{
  EXPECT_THROW((void)Hdl32eCalibration::loadFromYamlFile("/nonexistent/32db.yaml"), std::runtime_error);
}

TEST(Hdl32eCalibration, rejectsEmptyText)
{
  EXPECT_THROW((void)Hdl32eCalibration::loadFromYamlText(""), std::runtime_error);
}

TEST(Hdl32eCalibration, rejectsMissingRequiredKey)
{
  const std::string text =
    replaceAll(readStandardCalibrationText(), "dist_correction_x", "dist_correction_unknown");
  EXPECT_THROW((void)Hdl32eCalibration::loadFromYamlText(text), std::runtime_error);
}

TEST(Hdl32eCalibration, rejectsWrongLaserCount)
{
  const std::string text = replaceFirst(readStandardCalibrationText(), "num_lasers: 32", "num_lasers: 16");
  EXPECT_THROW((void)Hdl32eCalibration::loadFromYamlText(text), std::runtime_error);
}

TEST(Hdl32eCalibration, rejectsDuplicateLaserId)
{
  const std::string text = replaceFirst(readStandardCalibrationText(), "laser_id: 1,", "laser_id: 0,");
  EXPECT_THROW((void)Hdl32eCalibration::loadFromYamlText(text), std::runtime_error);
}

TEST(Hdl32eCalibration, rejectsNonPositiveDistanceResolution)
{
  const std::string text =
    replaceFirst(readStandardCalibrationText(), "distance_resolution: 0.002", "distance_resolution: 0");
  EXPECT_THROW((void)Hdl32eCalibration::loadFromYamlText(text), std::runtime_error);
}

// Laser 1 gets laser 0's angle to more digits than a float keeps: different as double, equal as float.
TEST(Hdl32eCalibration, rejectsVerticalAnglesThatAreEqualAsFloat)
{
  const std::string text = replaceFirst(
    readStandardCalibrationText(), "-0.1628392174657417", "-0.53529248158666");
  EXPECT_THROW((void)Hdl32eCalibration::loadFromYamlText(text), std::runtime_error);
}

}  // namespace
}  // namespace tsukuchalle_velodyne
