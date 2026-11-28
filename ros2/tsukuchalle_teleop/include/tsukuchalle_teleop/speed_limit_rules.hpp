#pragma once

#include <cmath>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tsukuchalle_teleop {

inline constexpr char kLinearLimitName[] = "scale_linear_limit.x";
inline constexpr char kAngularLimitName[] = "scale_angular_limit.yaw";
inline constexpr char kLinearScaleName[] = "scale_linear.x";
inline constexpr char kAngularScaleName[] = "scale_angular.yaw";
inline constexpr char kRequireEnableButtonName[] = "require_enable_button";
inline constexpr char kEnableTurboButtonName[] = "enable_turbo_button";

// Scales of the components this robot does not use (it drives with linear.x and angular.z only).
// The turbo x and yaw scales are not listed: the turbo button is refused at startup instead.
inline const std::vector<std::string> kUnusedScaleNames = {
    "scale_linear.y",       "scale_linear.z",       "scale_angular.pitch",       "scale_angular.roll",
    "scale_linear_turbo.y", "scale_linear_turbo.z", "scale_angular_turbo.pitch", "scale_angular_turbo.roll"};

struct SpeedLimits {
  double linear_x;
  double angular_yaw;
};

inline std::string format_number(double value) {
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

inline bool is_in_half_open_range(double value, double upper_limit) {
  return std::isfinite(value) && value > 0.0 && value <= upper_limit;
}

inline std::string range_message(const std::string& name, double upper_limit) {
  return name + " must be in (0, " + format_number(upper_limit) + "]";
}

// Reads a double parameter. Returns nullopt when it is not declared, not set or not a double.
inline std::optional<double> read_double(rclcpp::Node& node, const std::string& name) {
  rclcpp::Parameter parameter;
  if (!node.get_parameter(name, parameter) || parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return std::nullopt;
  }
  return parameter.as_double();
}

// Returns every reason the node must not start (empty when it may start).
inline std::vector<std::string> find_startup_violations(rclcpp::Node& node, SpeedLimits& limits_out) {
  std::vector<std::string> violations;
  const auto linear_limit = read_double(node, kLinearLimitName);
  const auto angular_limit = read_double(node, kAngularLimitName);
  if (!linear_limit || !std::isfinite(*linear_limit) || *linear_limit <= 0.0) {
    violations.push_back(std::string(kLinearLimitName) + " must be a finite double greater than 0");
  }
  if (!angular_limit || !std::isfinite(*angular_limit) || *angular_limit <= 0.0) {
    violations.push_back(std::string(kAngularLimitName) + " must be a finite double greater than 0");
  }
  if (!violations.empty()) {
    return violations;
  }
  limits_out = SpeedLimits{*linear_limit, *angular_limit};

  const auto linear_scale = read_double(node, kLinearScaleName);
  if (!linear_scale || !is_in_half_open_range(*linear_scale, limits_out.linear_x)) {
    violations.push_back(range_message(kLinearScaleName, limits_out.linear_x));
  }
  const auto angular_scale = read_double(node, kAngularScaleName);
  if (!angular_scale || !is_in_half_open_range(*angular_scale, limits_out.angular_yaw)) {
    violations.push_back(range_message(kAngularScaleName, limits_out.angular_yaw));
  }

  rclcpp::Parameter parameter;
  if (!node.get_parameter(kRequireEnableButtonName, parameter) ||
      parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL || !parameter.as_bool()) {
    violations.push_back(std::string(kRequireEnableButtonName) + " must be true");
  }
  if (!node.get_parameter(kEnableTurboButtonName, parameter) ||
      parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER || parameter.as_int() >= 0) {
    violations.push_back(std::string(kEnableTurboButtonName) + " must be negative (turbo is not allowed)");
  }
  for (const std::string& name : kUnusedScaleNames) {
    const auto scale = read_double(node, name);
    if (!scale || *scale != 0.0) {
      violations.push_back(name + " must be 0.0");
    }
  }
  return violations;
}

inline rcl_interfaces::msg::SetParametersResult make_result(bool is_successful, const std::string& reason) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = is_successful;
  result.reason = reason;
  return result;
}

// Accepts only a double in (0, limit] for the two speed scales and refuses every other change.
inline rcl_interfaces::msg::SetParametersResult judge_parameter_change(
    const SpeedLimits& limits, const std::vector<rclcpp::Parameter>& parameters) {
  for (const rclcpp::Parameter& parameter : parameters) {
    double upper_limit = 0.0;
    if (parameter.get_name() == kLinearScaleName) {
      upper_limit = limits.linear_x;
    } else if (parameter.get_name() == kAngularScaleName) {
      upper_limit = limits.angular_yaw;
    } else {
      return make_result(false, parameter.get_name() + " cannot be changed while running; edit the config file and restart");
    }
    const bool is_valid = parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                          is_in_half_open_range(parameter.as_double(), upper_limit);
    if (!is_valid) {
      return make_result(false, range_message(parameter.get_name(), upper_limit));
    }
  }
  return make_result(true, "");
}

}  // namespace tsukuchalle_teleop
