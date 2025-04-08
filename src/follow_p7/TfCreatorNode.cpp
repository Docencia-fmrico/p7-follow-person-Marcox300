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
#include "std_msgs/msg/int32.hpp"
#include <cmath>

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "vision_msgs/msg/detection3_d_array.hpp"

#include "follow_p7/TfCreatorNode.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace follow_p7
{

TfCreatorNode::TfCreatorNode()
: Node("tf_creator_node"),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
{
  this->declare_parameter("target", "person");
  this->get_parameter("target", target_);
  RCLCPP_INFO(get_logger(),"Te target is: %s", target_.c_str());

  subscriber_attractive_vector_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    "detection_3d", 10, std::bind(&TfCreatorNode::camera_callback, this, _1));

  timer_ = create_wall_timer(
    100ms, std::bind(&TfCreatorNode::control_cycle, this));
}

void
TfCreatorNode::control_cycle( )
{
  if (detections_3d_msg_.detections.empty() || this->now() - last_detection_time_ > 1s) {
    RCLCPP_WARN(get_logger(), "No detections available for transformation.");
    return;
  }

  for (auto & detection : detections_3d_msg_.detections) {
    if (!detection.results.empty()) {
      if (detection.results[0].hypothesis.class_id == target_) {
        geometry_msgs::msg::TransformStamped object_tf;
        object_tf.header.frame_id = "camera_rgb_optical_frame";
        object_tf.header.stamp = now();
        object_tf.child_frame_id = "detected_target";
        object_tf.transform.translation.x = detection.bbox.center.position.x;
        object_tf.transform.translation.y = detection.bbox.center.position.y;
        object_tf.transform.translation.z = detection.bbox.center.position.z;
        tf_broadcaster_->sendTransform(object_tf);

        RCLCPP_INFO(get_logger(), 
                    "Published camera_2object: x=%f, y=%f, z=%f", 
                    object_tf.transform.translation.x, 
                    object_tf.transform.translation.y, 
                    object_tf.transform.translation.z);
        break;
      }
    }
  }
}

void
TfCreatorNode::camera_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  if (msg->detections.empty()) {
    RCLCPP_INFO(get_logger(), "No camera detections received.");
    return;
  }
  RCLCPP_INFO(get_logger(), "Detección recibida");
  detections_3d_msg_ = *msg;
  last_detection_time_ = this->now();
}

}  //  namespace follow_p7
