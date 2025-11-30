// Copyright 2025 Jackson Huang
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

#include "loam_interface/loam_interface.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace loam_interface
{

LoamInterfaceNode::LoamInterfaceNode(const rclcpp::NodeOptions & options)
: Node("loam_interface", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("input_odom_topic", "/aft_mapped_to_init");
  this->declare_parameter<std::string>("output_odom_topic", "odom");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base");
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<bool>("publish_tf", true);

  // Get parameters
  this->get_parameter("input_odom_topic", input_odom_topic_);
  this->get_parameter("output_odom_topic", output_odom_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("publish_tf", publish_tf_);

  // Initialize TF2
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create publisher and subscriber
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    input_odom_topic_, 10,
    std::bind(&LoamInterfaceNode::odometryCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(), "Loam Interface Node started. Subscribing to %s, publishing to %s",
    input_odom_topic_.c_str(), output_odom_topic_.c_str());
}

void LoamInterfaceNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  // The input message has:
  // - header.frame_id = map_frame (where the pose is measured from)
  // - child_frame_id = base_frame (the robot base frame)
  // We republish it with odom_frame as the parent frame

  // Create output odometry message
  nav_msgs::msg::Odometry out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = odom_frame_;
  out.child_frame_id = base_frame_;

  // Copy pose and twist data
  out.pose = msg->pose;
  out.twist = msg->twist;

  // Publish odometry
  odom_pub_->publish(out);

  // Publish TF if enabled
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = msg->header.stamp;
    transform_stamped.header.frame_id = odom_frame_;
    transform_stamped.child_frame_id = base_frame_;

    transform_stamped.transform.translation.x = msg->pose.pose.position.x;
    transform_stamped.transform.translation.y = msg->pose.pose.position.y;
    transform_stamped.transform.translation.z = msg->pose.pose.position.z;
    transform_stamped.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(transform_stamped);
  }
}

}  // namespace loam_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(loam_interface::LoamInterfaceNode)
