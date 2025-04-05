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

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>

#include "follow_p7/LifeCycleControlFollow.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace follow_p7
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

LifeCycleControlFollow::LifeCycleControlFollow()
: LifecycleNode("life_cycle_follow"),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  vlin_pid_(0.5, 1.0, 0.0, 0.7),
  vrot_pid_(0.05, 1.0, 0.0, 1.0)
{
  publisher_velocity_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  is_publisher_active_ = false;
}

CallbackReturn
LifeCycleControlFollow::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Configuring...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
LifeCycleControlFollow::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Activating...");

  timer_ = create_wall_timer(
    100ms, std::bind(&LifeCycleControlFollow::timer_callback, this));

  publisher_velocity_->on_activate();
  velocity_.linear.x = 0;
  velocity_.angular.z = 0;
  publisher_velocity_->publish(velocity_);
  is_publisher_active_ = true;

  return CallbackReturn::SUCCESS;
}

CallbackReturn
LifeCycleControlFollow::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Deactivating...");

  deactivate_publisher();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
LifeCycleControlFollow::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Cleaning Up...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
LifeCycleControlFollow::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Shutting Down...");

  deactivate_publisher();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
LifeCycleControlFollow::on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Error State");

  return CallbackReturn::SUCCESS;
}

void
LifeCycleControlFollow::deactivate_publisher()
{
  if (is_publisher_active_) {
    timer_ = nullptr;
    velocity_.linear.x = 0;
    velocity_.angular.z = 0;
    publisher_velocity_->publish(velocity_);
    publisher_velocity_->on_deactivate();
    is_publisher_active_ = false;
    RCLCPP_INFO(get_logger(), "Stop velocity success");
  }
}

void
LifeCycleControlFollow::timer_callback()
{
  switch (state_) {
    case NOT_DETECT:
      RCLCPP_INFO(get_logger(), "Waitting to object");
      velocity_.angular.z = 0.2;
      publisher_velocity_->publish(velocity_);

      if (check_detections()) {
        velocity_.angular.z = 0;
        velocity_.linear.x = 0;
        publisher_velocity_->publish(velocity_);
        go_state(MOVE);
      }
      break;
    case MOVE:
      RCLCPP_INFO(get_logger(), "Moving to object");

      if (!check_detections()) {
        velocity_.linear.x = 0;
        velocity_.angular.z = 0;
        publisher_velocity_->publish(velocity_);
        go_state(NOT_DETECT);
      }
      break;
  }
}

void
LifeCycleControlFollow::go_state(int new_state)
{
  state_ = new_state;
}

double
LifeCycleControlFollow::select_vel(double x, double y)
{
  double dist = sqrt(x * x + y * y);
  RCLCPP_INFO(get_logger(), "DISTANCE = %f", dist);
  double vel_lin = std::clamp(vlin_pid_.get_output(dist - 1.0), -1.0, 1.0);
  RCLCPP_INFO(get_logger(), "Vel = %f", vel_lin);
  return vel_lin;
}

double
LifeCycleControlFollow::select_ang(double x, double y)
{
  double angle = atan2(y, x);
  RCLCPP_INFO(get_logger(), "ANGLE = %f", angle);
  double vel_rot = std::clamp(vrot_pid_.get_output(angle), -2.0, 2.0);
  RCLCPP_INFO(get_logger(), "Rot = %f", vel_rot);
  return vel_rot;
}

bool
LifeCycleControlFollow::check_detections()
{
  std::string error;
  if (tf_buffer_.canTransform("object_frame", "base_footprint", tf2::TimePointZero, &error)) {
    auto object = tf_buffer_.lookupTransform(
      "base_footprint", "object_frame", tf2::TimePointZero);

    double x = object.transform.translation.x;
    double y = object.transform.translation.y;
    velocity_.linear.x = select_vel(x, y);
    velocity_.angular.z = select_ang(x, y);
    publisher_velocity_->publish(velocity_);
    return true;

  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Error in TF base_footprint -> object [<< " << error << "]");
    return false;
  }
}

}  //  namespace follow_p7
