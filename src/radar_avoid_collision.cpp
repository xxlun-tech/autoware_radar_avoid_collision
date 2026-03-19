// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/radar_avoid_collision/radar_avoid_collision.hpp"

using rcl_interfaces::msg::ParameterType;

namespace autoware::radar_avoid_collision
{
RadarAvoidCollision::RadarAvoidCollision(const rclcpp::NodeOptions & options)
: rclcpp::Node("radar_avoid_collision", options),
  client_mode_switch_("/ap/mode_switch", this, true),
  client_arm_motors_("/ap/arm_motors", this, true)
{
  avoid_distance_ = declare_parameter<double>("avoid_distance", 25.0);
  avoid_angle_ = declare_parameter<double>("avoid_angle", 0.35);

  sub_radar_ = create_subscription<radar_msgs::msg::RadarScan>("input_radar_scan",
      // 10,
      rclcpp::QoS(10).best_effort(),
      std::bind(&RadarAvoidCollision::callback_radar, this, std::placeholders::_1));

  sub_status_ = create_subscription<ardupilot_msgs::msg::Status>("/ap/status",
      10,
      std::bind(&RadarAvoidCollision::callback_status, this, std::placeholders::_1));

  sub_avoid_collision_ = create_subscription<std_msgs::msg::Bool>("enable_avoid_collision",
      10,
      std::bind(&RadarAvoidCollision::callback_avoid_collision, this, std::placeholders::_1));

  timer_ = create_timer(std::chrono::duration<double>(1.0), std::bind(&RadarAvoidCollision::avoid_collision_status, this));
  pub_avoid_collision_ = create_publisher<std_msgs::msg::Bool>("avoid_collision_status", rclcpp::QoS{10});

  // Add callback for dynamic parameters
  dyn_params_handler_ = add_on_set_parameters_callback(
    std::bind(
      &RadarAvoidCollision::dynamicParametersCallback,
      this, std::placeholders::_1));
}

void RadarAvoidCollision::avoid_collision_status()
{
  std_msgs::msg::Bool status;
  status.data = avoid_collision_enabled_;
  pub_avoid_collision_->publish(status);
}

void RadarAvoidCollision::callback_avoid_collision(const std_msgs::msg::Bool::SharedPtr msg)
{
  avoid_collision_enabled_ = msg->data;
}

void RadarAvoidCollision::callback_status(const ardupilot_msgs::msg::Status::SharedPtr msg)
{
  if (msg->mode == 15) vehicle_stop_ = false;
  if (!vehicle_stop_ || msg->mode == 0) vehicle_mode_ = msg->mode;
}

void RadarAvoidCollision::callback_radar(const radar_msgs::msg::RadarScan::SharedPtr msg)
{
  if (!avoid_collision_enabled_) return;

  if (vehicle_mode_ != 15) return;

  if (!client_mode_switch_.wait_for_service(std::chrono::milliseconds(10))) {
    RCLCPP_WARN(get_logger(), "Ardupilot mode change service is not available!");
    return;
  }

  bool should_stop = false;
  for (const radar_msgs::msg::RadarReturn & value : msg->returns)
  {
    if (value.range > avoid_distance_) continue; 
    if (std::abs(value.azimuth) > avoid_angle_) continue; 
    should_stop = true;
    break;
  }

  RCLCPP_INFO(get_logger(), "Should stop : %s", should_stop ? "true" : "false");

  if (should_stop && !vehicle_stop_) 
  {
    auto req = std::make_shared<ardupilot_msgs::srv::ModeSwitch::Request>();
    req->mode = 5; // 4 hold, 5 loiter

    auto res = client_mode_switch_.invoke(req);
    if (res->status) {
      vehicle_stop_ = true;
      RCLCPP_INFO(get_logger(), "Radar mode switch to loiter success");
    } else {
      RCLCPP_INFO(get_logger(), "Radar mode switch to loiter failed");
    }
  }
}

/**
  * @brief Callback executed when a parameter change is detected
  * @param event ParameterEvent message
  */
rcl_interfaces::msg::SetParametersResult
RadarAvoidCollision::dynamicParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == "avoid_distance") {
        avoid_distance_ = parameter.as_double();
        break;
      }
    }
  }
  result.successful = true;
  return result;
}
}  // namespace autoware::radar_avoid_collision

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::radar_avoid_collision::RadarAvoidCollision)
