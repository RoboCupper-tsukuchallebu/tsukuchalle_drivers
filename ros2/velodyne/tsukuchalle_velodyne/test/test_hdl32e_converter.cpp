#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/time.hpp>

#include "cloud_cell_reader.hpp"
#include "synthetic_packets.hpp"
#include "tsukuchalle_velodyne/hdl32e_converter.hpp"

namespace tsukuchalle_velodyne
{
namespace
{

using test::Cell;
using test::makeRandomScan;
using test::makeUniformScan;
using test::readCell;

constexpr double DEFAULT_MIN_RANGE = 0.9;
constexpr double DEFAULT_MAX_RANGE = 130.0;
constexpr std::size_t EXPECTED_WIDTH = 32;
constexpr std::size_t EXPECTED_POINT_STEP = 22;
constexpr float DISTANCE_TOLERANCE_METERS = 1e-3f;

const std::string STANDARD_CALIBRATION_PATH =
  std::string(TSUKUCHALLE_VELODYNE_CONFIG_DIRECTORY) + "/32db.yaml";

Hdl32eConverter makeConverter(
  const double min_range = DEFAULT_MIN_RANGE, const double max_range = DEFAULT_MAX_RANGE)
{
  return Hdl32eConverter(
    Hdl32eCalibration::loadFromYamlFile(STANDARD_CALIBRATION_PATH), min_range, max_range);
}

// Calibration whose ring equals the laser id, with one value for every laser.
std::string makeYamlText(
  const float dist_correction, const float focal_slope, const int min_intensity,
  const int max_intensity)
{
  std::ostringstream text;
  text << "num_lasers: 32\ndistance_resolution: 0.002\nlasers:\n";
  for (int laser_index = 0; laser_index < LASER_COUNT; ++laser_index) {
    text << "- {laser_id: " << laser_index << ", rot_correction: 0.0, vert_correction: "
         << (laser_index - 16) * 0.02 << ", dist_correction: " << dist_correction
         << ", dist_correction_x: " << dist_correction << ", dist_correction_y: " << dist_correction
         << ", vert_offset_correction: 0.0, focal_distance: 0.0, focal_slope: " << focal_slope
         << ", min_intensity: " << min_intensity << ", max_intensity: " << max_intensity << "}\n";
  }
  return text.str();
}

std::size_t countFiniteCells(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::size_t finite_count = 0;
  for (std::size_t row = 0; row < cloud.height; ++row) {
    for (std::size_t column = 0; column < cloud.width; ++column) {
      finite_count += readCell(cloud, row, column).isFinite() ? 1 : 0;
    }
  }
  return finite_count;
}

TEST(Hdl32eConverter, producesOrganizedShapeFor181Packets)
{
  const auto scan = makeUniformScan(test::PACKETS_PER_REVOLUTION, 5.0);

  const ConversionResult result = makeConverter().convert(scan);
  const auto & cloud = *result.cloud;

  EXPECT_EQ(cloud.width, 32U);
  EXPECT_EQ(cloud.height, 2172U);
  EXPECT_EQ(cloud.point_step, 22U);
  EXPECT_EQ(cloud.row_step, 704U);
  EXPECT_EQ(cloud.data.size(), 1529088U);
  EXPECT_FALSE(cloud.is_bigendian);
  EXPECT_FALSE(cloud.is_dense);
  EXPECT_EQ(cloud.header.frame_id, "velodyne");
  EXPECT_EQ(cloud.header.stamp.sec, scan.header.stamp.sec);
  EXPECT_EQ(cloud.header.stamp.nanosec, scan.header.stamp.nanosec);
  EXPECT_EQ(result.invalid_block_count, 0U);
}

TEST(Hdl32eConverter, describesFieldsInOrder)
{
  const auto cloud = makeConverter().convert(makeUniformScan(1, 5.0)).cloud;

  ASSERT_EQ(cloud->fields.size(), 6U);
  const char * expected_names[] = {"x", "y", "z", "intensity", "ring", "time"};
  const std::uint32_t expected_offsets[] = {0, 4, 8, 12, 16, 18};
  const std::uint8_t expected_types[] = {
    sensor_msgs::msg::PointField::FLOAT32, sensor_msgs::msg::PointField::FLOAT32,
    sensor_msgs::msg::PointField::FLOAT32, sensor_msgs::msg::PointField::FLOAT32,
    sensor_msgs::msg::PointField::UINT16, sensor_msgs::msg::PointField::FLOAT32};
  for (std::size_t field_index = 0; field_index < 6; ++field_index) {
    EXPECT_EQ(cloud->fields[field_index].name, expected_names[field_index]);
    EXPECT_EQ(cloud->fields[field_index].offset, expected_offsets[field_index]);
    EXPECT_EQ(cloud->fields[field_index].datatype, expected_types[field_index]);
    EXPECT_EQ(cloud->fields[field_index].count, 1U);
  }
}

TEST(Hdl32eConverter, singlePacketGivesTwelveRows)
{
  const auto cloud = makeConverter().convert(makeUniformScan(1, 5.0)).cloud;

  EXPECT_EQ(cloud->height, 12U);
  EXPECT_EQ(cloud->data.size(), 22U * 32U * 12U);
}

TEST(Hdl32eConverter, emptyScanGivesEmptyCloud)
{
  const auto cloud = makeConverter().convert(makeUniformScan(0, 5.0)).cloud;

  EXPECT_EQ(cloud->height, 0U);
  EXPECT_TRUE(cloud->data.empty());
}

TEST(Hdl32eConverter, uniformFiveMetersGivesAllFinitePointsOnTheSphere)
{
  const auto cloud = makeConverter().convert(makeUniformScan(test::PACKETS_PER_REVOLUTION, 5.0)).cloud;

  EXPECT_EQ(countFiniteCells(*cloud), 69504U);
  std::set<std::uint16_t> rings_seen;
  for (std::size_t row = 0; row < cloud->height; ++row) {
    for (std::size_t column = 0; column < cloud->width; ++column) {
      const Cell cell = readCell(*cloud, row, column);
      EXPECT_NEAR(cell.distanceFromOrigin(), 5.0f, DISTANCE_TOLERANCE_METERS);
      EXPECT_EQ(cell.ring, column);
      rings_seen.insert(cell.ring);
    }
  }
  EXPECT_EQ(rings_seen.size(), EXPECTED_WIDTH);
}

// Azimuth 90 degrees is velodyne +X (right), which is ROS -Y. Laser 31 looks up by vert_correction.
TEST(Hdl32eConverter, placesAPointInRosCoordinates)
{
  auto scan = makeUniformScan(1, 5.0);
  test::setBlockAzimuth(scan.packets[0], 0, 9000);
  const Hdl32eCalibration calibration = Hdl32eCalibration::loadFromYamlFile(STANDARD_CALIBRATION_PATH);
  const double vert_angle = calibration.laser_corrections()[31].vert_correction;
  const Hdl32eConverter converter(calibration, DEFAULT_MIN_RANGE, DEFAULT_MAX_RANGE);

  const auto cloud = converter.convert(scan).cloud;
  const Cell cell = readCell(*cloud, 0, 31);

  EXPECT_NEAR(cell.x, 0.0, 1e-4);
  EXPECT_NEAR(cell.y, -5.0 * std::cos(vert_angle), 1e-4);
  EXPECT_NEAR(cell.z, 5.0 * std::sin(vert_angle), 1e-4);
  EXPECT_EQ(cell.ring, 31U);
}

TEST(Hdl32eConverter, azimuthZeroPointsForward)
{
  auto scan = makeUniformScan(1, 5.0);
  test::setBlockAzimuth(scan.packets[0], 0, 0);

  const Cell cell = readCell(*makeConverter().convert(scan).cloud, 0, 31);

  EXPECT_GT(cell.x, 4.9f);
  EXPECT_NEAR(cell.y, 0.0f, 1e-4f);
}

TEST(Hdl32eConverter, timeIsFiringOffsetPlusPacketOffset)
{
  const auto scan = makeUniformScan(test::PACKETS_PER_REVOLUTION, 5.0);
  const auto cloud = makeConverter().convert(scan).cloud;

  for (const std::size_t packet_index : {std::size_t{0}, std::size_t{90}, std::size_t{180}}) {
    const double packet_offset_seconds =
      (rclcpp::Time(scan.packets[packet_index].stamp) - rclcpp::Time(scan.header.stamp)).seconds();
    for (const int block_index : {0, 5, 11}) {
      for (const int laser_index : {0, 1, 30, 31}) {
        const int ring = Hdl32eCalibration::loadFromYamlFile(STANDARD_CALIBRATION_PATH)
          .laser_corrections()[laser_index].laser_ring;
        const double firing_offset_seconds = FULL_FIRING_CYCLE_SECONDS * block_index +
          SINGLE_FIRING_SECONDS * (laser_index / 2);
        const Cell cell = readCell(*cloud, packet_index * BLOCKS_PER_PACKET + block_index, ring);
        EXPECT_NEAR(cell.time, firing_offset_seconds + packet_offset_seconds, 1e-6)
          << "packet " << packet_index << " block " << block_index << " laser " << laser_index;
      }
    }
  }
}

TEST(Hdl32eConverter, noReturnGivesNanButKeepsRingAndTime)
{
  auto scan = makeUniformScan(1, 5.0);
  test::setLaser(scan.packets[0], 2, 7, 0, 50);

  const auto cloud = makeConverter().convert(scan).cloud;
  const int ring = Hdl32eCalibration::loadFromYamlFile(STANDARD_CALIBRATION_PATH)
    .laser_corrections()[7].laser_ring;
  const Cell cell = readCell(*cloud, 2, ring);

  EXPECT_TRUE(cell.isAllNan());
  EXPECT_EQ(cell.ring, ring);
  EXPECT_TRUE(std::isfinite(cell.time));
  EXPECT_EQ(countFiniteCells(*cloud), 12U * 32U - 1U);
}

TEST(Hdl32eConverter, outOfRangeGivesNanButKeepsRingAndTime)
{
  auto scan = makeUniformScan(1, 5.0);
  test::setLaser(scan.packets[0], 0, 0, 440, 50);    // 0.88 m
  test::setLaser(scan.packets[0], 0, 1, 65500, 50);  // 131 m

  const auto cloud = makeConverter().convert(scan).cloud;

  for (const int laser_index : {0, 1}) {
    const int ring = Hdl32eCalibration::loadFromYamlFile(STANDARD_CALIBRATION_PATH)
      .laser_corrections()[laser_index].laser_ring;
    const Cell cell = readCell(*cloud, 0, ring);
    EXPECT_TRUE(cell.isAllNan()) << "laser " << laser_index;
    EXPECT_EQ(cell.ring, ring);
    EXPECT_TRUE(std::isfinite(cell.time));
  }
}

TEST(Hdl32eConverter, rangeLimitsMoveTheBoundary)
{
  const auto scan = makeUniformScan(1, 5.0);

  EXPECT_EQ(countFiniteCells(*makeConverter(5.5, 130.0).convert(scan).cloud), 0U);
  EXPECT_EQ(countFiniteCells(*makeConverter(0.9, 4.5).convert(scan).cloud), 0U);
  EXPECT_EQ(countFiniteCells(*makeConverter(4.5, 5.5).convert(scan).cloud), 12U * 32U);
}

// The reference compares the float distance with the double limits, closed on both ends.
TEST(Hdl32eConverter, rangeBoundaryIsClosed)
{
  constexpr std::uint16_t RAW_AT_BOUNDARY = 450;
  const float distance_at_boundary = static_cast<float>(RAW_AT_BOUNDARY) * 0.002f;
  auto scan = makeUniformScan(1, 5.0);
  test::setLaser(scan.packets[0], 0, 0, RAW_AT_BOUNDARY, 10);
  const int ring = Hdl32eCalibration::loadFromYamlFile(STANDARD_CALIBRATION_PATH)
    .laser_corrections()[0].laser_ring;

  const double exact = static_cast<double>(distance_at_boundary);
  const double just_above = std::nextafter(exact, std::numeric_limits<double>::infinity());
  const double just_below = std::nextafter(exact, 0.0);

  EXPECT_TRUE(readCell(*makeConverter(exact, 130.0).convert(scan).cloud, 0, ring).isFinite());
  EXPECT_TRUE(readCell(*makeConverter(just_below, 130.0).convert(scan).cloud, 0, ring).isFinite());
  EXPECT_TRUE(readCell(*makeConverter(just_above, 130.0).convert(scan).cloud, 0, ring).isAllNan());
  EXPECT_TRUE(readCell(*makeConverter(0.1, exact).convert(scan).cloud, 0, ring).isFinite());
  EXPECT_TRUE(readCell(*makeConverter(0.1, just_below).convert(scan).cloud, 0, ring).isAllNan());
}

TEST(Hdl32eConverter, appliesDistanceCorrection)
{
  const Hdl32eConverter converter(
    Hdl32eCalibration::loadFromYamlText(makeYamlText(0.25f, 0.0f, 0, 255)), DEFAULT_MIN_RANGE,
    DEFAULT_MAX_RANGE);

  const Cell cell = readCell(*converter.convert(makeUniformScan(1, 5.0)).cloud, 0, 16);

  EXPECT_NEAR(cell.distanceFromOrigin(), 5.25f, DISTANCE_TOLERANCE_METERS);
}

TEST(Hdl32eConverter, clampsIntensity)
{
  const auto scan = makeUniformScan(1, 5.0, 100);
  const auto make = [](const int min_intensity, const int max_intensity) {
      return Hdl32eConverter(
        Hdl32eCalibration::loadFromYamlText(makeYamlText(0.0f, 0.0f, min_intensity, max_intensity)),
        DEFAULT_MIN_RANGE, DEFAULT_MAX_RANGE);
    };

  EXPECT_EQ(readCell(*make(0, 255).convert(scan).cloud, 0, 16).intensity, 100.0f);
  EXPECT_EQ(readCell(*make(0, 60).convert(scan).cloud, 0, 16).intensity, 60.0f);
  EXPECT_EQ(readCell(*make(130, 255).convert(scan).cloud, 0, 16).intensity, 130.0f);
}

TEST(Hdl32eConverter, addsFocalCorrectionToIntensity)
{
  const auto scan = makeUniformScan(1, 5.0, 100);
  const Hdl32eConverter converter(
    Hdl32eCalibration::loadFromYamlText(makeYamlText(0.0f, 0.5f, 0, 255)), DEFAULT_MIN_RANGE,
    DEFAULT_MAX_RANGE);

  // focal_distance 0 gives focal_offset 256; the squared distance term is negligible at 5 m.
  EXPECT_NEAR(readCell(*converter.convert(scan).cloud, 0, 16).intensity, 100.0f + 0.5f * 256.0f, 0.1f);
}

TEST(Hdl32eConverter, invalidBlocksBecomeNanRowsAndOthersAreUntouched)
{
  const auto valid_scan = makeUniformScan(2, 5.0);
  const auto valid_cloud = makeConverter().convert(valid_scan).cloud;
  auto scan = valid_scan;
  test::setBlockHeader(scan.packets[0], 1, 0xDDFF);
  test::setBlockHeader(scan.packets[0], 2, 0x1234);
  test::setBlockAzimuth(scan.packets[0], 3, 36000);
  test::setBlockAzimuth(scan.packets[1], 0, 0xFFFF);

  const ConversionResult result = makeConverter().convert(scan);
  const auto & cloud = *result.cloud;

  EXPECT_EQ(result.invalid_block_count, 4U);
  EXPECT_EQ(cloud.height, valid_cloud->height);
  EXPECT_EQ(cloud.data.size(), valid_cloud->data.size());
  const std::size_t invalid_rows[] = {1, 2, 3, 12};
  for (std::size_t row = 0; row < cloud.height; ++row) {
    const bool is_invalid = std::find(std::begin(invalid_rows), std::end(invalid_rows), row) !=
      std::end(invalid_rows);
    for (std::size_t column = 0; column < cloud.width; ++column) {
      const Cell cell = readCell(cloud, row, column);
      const Cell valid_cell = readCell(*valid_cloud, row, column);
      EXPECT_EQ(cell.ring, column);
      EXPECT_EQ(cell.time, valid_cell.time);
      if (is_invalid) {
        EXPECT_TRUE(cell.isAllNan()) << row << "," << column;
      } else {
        EXPECT_EQ(cell.x, valid_cell.x);
        EXPECT_EQ(cell.intensity, valid_cell.intensity);
      }
    }
  }
}

TEST(Hdl32eConverter, randomScanHasTheExpectedMixOfNanAndFinite)
{
  const auto cloud = makeConverter().convert(makeRandomScan(test::PACKETS_PER_REVOLUTION, 1)).cloud;

  const double finite_ratio = static_cast<double>(countFiniteCells(*cloud)) / (32.0 * 2172.0);
  EXPECT_NEAR(finite_ratio, 0.55, 0.02);
}

}  // namespace
}  // namespace tsukuchalle_velodyne
