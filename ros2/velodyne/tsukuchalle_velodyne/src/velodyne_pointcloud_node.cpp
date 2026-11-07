// velodyne_packets (VelodyneScan) -> velodyne_points (PointCloud2) for the HDL-32E.
// Replaces ros-drivers' velodyne_transform_node without PCL. Parameters: calibration (required),
// min_range, max_range.
#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include "tsukuchalle_velodyne/hdl32e_converter.hpp"

namespace tsukuchalle_velodyne
{
namespace
{

constexpr double DEFAULT_MIN_RANGE_METERS = 0.9;
constexpr double DEFAULT_MAX_RANGE_METERS = 130.0;
constexpr int WARNING_INTERVAL_MILLISECONDS = 60000;
constexpr std::size_t QUEUE_DEPTH = 10;

class VelodynePointcloudNode final : public rclcpp::Node
{
public:
  VelodynePointcloudNode()
  : rclcpp::Node("velodyne_pointcloud_node"),
    converter_(makeConverter())
  {
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points", QUEUE_DEPTH);
    subscription_ = create_subscription<velodyne_msgs::msg::VelodyneScan>(
      "velodyne_packets", QUEUE_DEPTH,
      [this](const velodyne_msgs::msg::VelodyneScan::ConstSharedPtr scan) {processScan(*scan);});
    RCLCPP_INFO(get_logger(), "converting velodyne_packets to velodyne_points");
  }

private:
  Hdl32eConverter makeConverter()
  {
    const std::string calibration_path = declare_parameter("calibration", std::string());
    const double min_range = declare_parameter("min_range", DEFAULT_MIN_RANGE_METERS);
    const double max_range = declare_parameter("max_range", DEFAULT_MAX_RANGE_METERS);
    if (calibration_path.empty()) {
      throw std::runtime_error("parameter 'calibration' is required (path of the calibration yaml)");
    }
    if (!(0.0 < min_range && min_range < max_range)) {
      throw std::runtime_error("parameters must satisfy 0 < min_range < max_range");
    }
    RCLCPP_INFO(get_logger(), "calibration: %s", calibration_path.c_str());
    return Hdl32eConverter(Hdl32eCalibration::loadFromYamlFile(calibration_path), min_range, max_range);
  }

  void processScan(const velodyne_msgs::msg::VelodyneScan & scan)
  {
    if (publisher_->get_subscription_count() == 0 &&
      publisher_->get_intra_process_subscription_count() == 0)
    {
      return;
    }
    if (scan.packets.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), WARNING_INTERVAL_MILLISECONDS, "received a scan without packets");
      return;
    }
    ConversionResult result = converter_.convert(scan);
    if (result.invalid_block_count > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), WARNING_INTERVAL_MILLISECONDS,
        "%zu blocks with an invalid header or azimuth were turned into NaN rows",
        result.invalid_block_count);
    }
    publisher_->publish(std::move(result.cloud));
  }

  Hdl32eConverter converter_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<velodyne_msgs::msg::VelodyneScan>::SharedPtr subscription_;
};

}  // namespace
}  // namespace tsukuchalle_velodyne

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<tsukuchalle_velodyne::VelodynePointcloudNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("velodyne_pointcloud_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
