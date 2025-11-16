// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

class PoseTfBroadcaster : public rclcpp::Node {
public:
  PoseTfBroadcaster()
  : rclcpp::Node("pose_tf_broadcaster")
  {
    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    auto tf_qos = rclcpp::QoS(rclcpp::KeepLast(100))
        .best_effort()
        .durability_volatile();

    sub_pose_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "pose", tf_qos,
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        tf_br_->sendTransform(msg->transforms);
      });

    sub_pose_static_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "pose_static", tf_qos,
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        static_br_->sendTransform(msg->transforms);
      });
  }

private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_br_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_pose_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_pose_static_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}