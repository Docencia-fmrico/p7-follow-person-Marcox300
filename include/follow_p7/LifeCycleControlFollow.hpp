// Copyright 2025 Marcos
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

#ifndef MOVE_P7__LIFECYCLECONTROLFOLLOW_HPP_
#define MOVE_P7__LIFECYCLECONTROLFOLLOW_HPP_

#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/macros.hpp"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"

#include "follow_p7/PIDController.hpp"

namespace follow_p7
{

class LifeCycleControlFollow : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(LifeCycleControlFollow)

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  LifeCycleControlFollow();
  void deactivate_publisher();
  void timer_callback();
  bool check_detections();
  double select_vel(double x, double y);
  double select_ang(double x, double y);
  void go_state(int new_state);

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state);

private:
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr publisher_velocity_;
  bool is_publisher_active_;

  static const int NOT_DETECT = 0;
  static const int MOVE = 1;
  int state_;

  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist velocity_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  PIDController vlin_pid_, vrot_pid_;
};

}  //  namespace follow_p7


#endif  // MOVE_P7__LIFECYCLECONTROLFOLLOW_HPP_
