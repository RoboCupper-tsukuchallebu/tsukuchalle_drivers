// teleop_twist_joy with a speed limit. The speed calculation and the deadman logic are the stock
// teleop_twist_joy::TeleopTwistJoy; this file only adds checks. See tsukuchalle_drivers#38.
#include <memory>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "teleop_twist_joy/teleop_twist_joy.hpp"
#include "tsukuchalle_teleop/speed_limit_rules.hpp"

namespace {

constexpr int kExitFailure = 1;

// Declared before our callback is registered: declaring also runs the registered callbacks, and ours
// would refuse every name but the two scales. The stock callback ignores names it does not know.
bool declare_read_only_limits(rclcpp::Node& node) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  for (const char* name : {tsukuchalle_teleop::kLinearLimitName, tsukuchalle_teleop::kAngularLimitName}) {
    try {
      node.declare_parameter(name, rclcpp::PARAMETER_DOUBLE, descriptor);
    } catch (const std::exception& error) {
      RCLCPP_ERROR(node.get_logger(), "cannot declare %s: %s", name, error.what());
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<teleop_twist_joy::TeleopTwistJoy>(rclcpp::NodeOptions());

  if (!declare_read_only_limits(*node)) {
    rclcpp::shutdown();
    return kExitFailure;
  }
  tsukuchalle_teleop::SpeedLimits limits{0.0, 0.0};
  const std::vector<std::string> violations = tsukuchalle_teleop::find_startup_violations(*node, limits);
  if (!violations.empty()) {
    for (const std::string& violation : violations) {
      RCLCPP_ERROR(node->get_logger(), "refusing to start: %s", violation.c_str());
    }
    rclcpp::shutdown();
    return kExitFailure;
  }

  // Registered after the stock callback, so it runs first and a refusal stops the chain.
  rclcpp::Node& node_reference = *node;
  const auto callback_handle = node->add_on_set_parameters_callback(
      [&node_reference, limits](const std::vector<rclcpp::Parameter>& parameters) {
        const auto result = tsukuchalle_teleop::judge_parameter_change(limits, parameters);
        for (const rclcpp::Parameter& parameter : parameters) {
          if (!result.successful) {
            RCLCPP_WARN(node_reference.get_logger(), "refused %s: %s", parameter.get_name().c_str(),
                        result.reason.c_str());
            continue;
          }
          const auto previous = tsukuchalle_teleop::read_double(node_reference, parameter.get_name());
          RCLCPP_INFO(node_reference.get_logger(), "%s: %s -> %s", parameter.get_name().c_str(),
                      previous ? tsukuchalle_teleop::format_number(*previous).c_str() : "unset",
                      tsukuchalle_teleop::format_number(parameter.as_double()).c_str());
        }
        return result;
      });

  RCLCPP_INFO(node->get_logger(), "speed limits: linear.x %s m/s, angular.yaw %s rad/s",
              tsukuchalle_teleop::format_number(limits.linear_x).c_str(),
              tsukuchalle_teleop::format_number(limits.angular_yaw).c_str());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
