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

#ifndef AUTOWARE__RADAR_AVOID_COLLISION__RADAR_AVOID_COLLISION_HPP_
#define AUTOWARE__RADAR_AVOID_COLLISION__RADAR_AVOID_COLLISION_HPP_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <radar_msgs/msg/radar_scan.hpp>
#include <ardupilot_msgs/srv/mode_switch.hpp>
#include <ardupilot_msgs/srv/arm_motors.hpp>
#include <ardupilot_msgs/msg/status.hpp>

#include "autoware/radar_avoid_collision/service_client.hpp"

namespace autoware::radar_avoid_collision
{
class RadarAvoidCollision : public rclcpp::Node
{
public:
  explicit RadarAvoidCollision(const rclcpp::NodeOptions & options);

protected:
  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

private:
  void callback_radar(const radar_msgs::msg::RadarScan::SharedPtr msg);
  rclcpp::Subscription<radar_msgs::msg::RadarScan>::SharedPtr sub_radar_;

  void callback_status(const ardupilot_msgs::msg::Status::SharedPtr msg);
  rclcpp::Subscription<ardupilot_msgs::msg::Status>::SharedPtr sub_status_;

  void callback_avoid_collision(const std_msgs::msg::Bool::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_avoid_collision_;

  void avoid_collision_status(); 
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_avoid_collision_;

  nav2::ServiceClient<ardupilot_msgs::srv::ModeSwitch> client_mode_switch_;
  nav2::ServiceClient<ardupilot_msgs::srv::ArmMotors> client_arm_motors_;

  double avoid_distance_ = 25.0;
  double avoid_angle_ = 0.35;

  int vehicle_mode_ = 0;
  bool vehicle_stop_ = false;

  bool avoid_collision_enabled_ = false;
};
}  // namespace autoware::radar_avoid_collision

#endif  // AUTOWARE__RADAR_AVOID_COLLISION__RADAR_AVOID_COLLISION_HPP_
