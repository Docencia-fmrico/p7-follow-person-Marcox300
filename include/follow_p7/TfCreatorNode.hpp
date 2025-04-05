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

#ifndef MOVE_P7__TFCREATORNODE_HPP_
#define MOVE_P7__TFCREATORNODE_HPP_

#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


namespace follow_p7
{

class TfCreatorNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(TfCreatorNode)

  TfCreatorNode();
  void control_cycle();
  void camera_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);

private:
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr subscriber_attractive_vector_;
  vision_msgs::msg::Detection3DArray detections_3d_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_detection_time_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string target_;
};

}  //  namespace follow_p7

#endif  // MOVE_P7__TFCREATORNODE_HPP_
