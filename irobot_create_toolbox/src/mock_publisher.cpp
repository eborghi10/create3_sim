// Copyright 2021 iRobot, Inc.
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
//
// @author Lola Segura (lsegura@irobot.com)

#include <irobot_create_toolbox/mock_publisher.hpp>

namespace irobot_create_toolbox
{
MockPublisher::MockPublisher()
: rclcpp::Node("mock_publisher_node")
{
  // Topic parameter to publish buttons to
  buttons_publisher_topic_ = declare_and_get_parameter<std::string>("button_topic", this);
  // Topic parameter to publish slip status to
  slip_status_publisher_topic_ = declare_and_get_parameter<std::string>("slip_status_topic", this);
  // Topic parameter to publish kidnap status to
  kidnap_status_publisher_topic_ = declare_and_get_parameter<std::string>("kidnap_status_topic", this);
  // Topic parameter to publish battery state to
  battery_state_publisher_topic_ = declare_and_get_parameter<std::string>("battery_state_topic", this);
  // Topic parameter to publish stop status to
  stop_status_publisher_topic_ = declare_and_get_parameter<std::string>("stop_status_topic", this);

  // Subscriber topics
  hazard_subscription_topic = declare_and_get_parameter<std::string>("hazard_topic", this);
  wheel_vels_subscription_topic = declare_and_get_parameter<std::string>("wheel_vels_topic", this);
  lightring_subscription_topic = declare_and_get_parameter<std::string>("lightring_topic", this);

  // Publish rate parameters
  const double buttons_publish_rate = declare_and_get_parameter<double>("buttons_publish_rate", this);  // Hz
  const double slip_status_publish_rate = declare_and_get_parameter<double>("slip_status_publish_rate", this);  // Hz
  const double kidnap_status_publish_rate = declare_and_get_parameter<double>("kidnap_status_publish_rate", this);  // Hz
  const double battery_state_publish_rate = declare_and_get_parameter<double>("battery_state_publish_rate", this);  // Hz

  // Define buttons publisher
  buttons_publisher_ = create_publisher<irobot_create_msgs::msg::InterfaceButtons>(
    buttons_publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << buttons_publisher_topic_);

  // Define slip status publisher
  slip_status_publisher_ = create_publisher<irobot_create_msgs::msg::SlipStatus>(
    slip_status_publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << slip_status_publisher_topic_);

  // Define kidnap status publisher
  kidnap_status_publisher_ = create_publisher<irobot_create_msgs::msg::KidnapStatus>(
    kidnap_status_publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << kidnap_status_publisher_topic_);

  // Define battery state publisher
  battery_state_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(
    battery_state_publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << battery_state_publisher_topic_);

  // Define stop status publisher
  stop_status_publisher_ = create_publisher<irobot_create_msgs::msg::StopStatus>(
    stop_status_publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << stop_status_publisher_topic_);

 // Subscription to the hazard detection vector
  kidnap_status_subscription_ = create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
    hazard_subscription_topic, rclcpp::SensorDataQoS(), std::bind(&MockPublisher::kidnap_callback, this, std::placeholders::_1));
  RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << hazard_subscription_topic);
  // Subscription to the stop status
  stop_status_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    wheel_vels_subscription_topic, rclcpp::SensorDataQoS(), std::bind(&MockPublisher::stop_callback, this, std::placeholders::_1));
  RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << wheel_vels_subscription_topic);

   // Subscription to the lightring leds
  lightring_subscription_ = create_subscription<irobot_create_msgs::msg::LightringLeds>(
    lightring_subscription_topic, rclcpp::SensorDataQoS(), std::bind(&MockPublisher::lightring_callback, this, std::placeholders::_1));
  RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << lightring_subscription_topic);

  buttons_timer_ = create_wall_timer(std::chrono::duration<double>(1 / buttons_publish_rate), [this]() {
    // Set header timestamp.
    this->buttons_msg_.header.stamp = now();

    this->buttons_msg_.button_1.header.stamp = now();
    this->buttons_msg_.button_power.header.stamp = now();
    this->buttons_msg_.button_2.header.stamp = now();

    // Publish topics
    this->buttons_publisher_->publish(this->buttons_msg_);
  });

  slip_status_timer_ = create_wall_timer(std::chrono::duration<double>(1 / slip_status_publish_rate), [this]() {
    // Set header timestamp.
    this->slip_status_msg_.header.stamp = now();

    // Publish topics
    this->slip_status_publisher_->publish(this->slip_status_msg_);
  });

  kidnap_status_timer_ = create_wall_timer(std::chrono::duration<double>(1 / kidnap_status_publish_rate), [this]() {
    // Set header timestamp.
    this->kidnap_status_msg_.header.stamp = now();

    // Set kidnap status.
    this->kidnap_status_msg_.is_kidnapped = kidnap_status_;

    // Publish topics
    this->kidnap_status_publisher_->publish(this->kidnap_status_msg_);
  });

  battery_state_timer_  = create_wall_timer(std::chrono::duration<double>(1 / battery_state_publish_rate), [this]() {
    // Set header timestamp.
    this->battery_state_msg_.header.stamp = now();

    this->battery_state_msg_.percentage = 1;

    // Publish topics
    this->battery_state_publisher_->publish(this->battery_state_msg_);
  });

  // Set buttons header
  this->buttons_msg_.header.frame_id = "base_link";
  this->buttons_msg_.button_1.header.frame_id = "button_1";
  this->buttons_msg_.button_power.header.frame_id = "button_power";
  this->buttons_msg_.button_2.header.frame_id = "button_2";
  // Set buttons pressed state
  this->buttons_msg_.button_1.is_pressed = false;
  this->buttons_msg_.button_power.is_pressed = false;
  this->buttons_msg_.button_2.is_pressed = false;

  // Set slip status header
  this->slip_status_msg_.header.frame_id = "base_link";
  // Set slip status status
  this->slip_status_msg_.is_slipping = false;
}

void MockPublisher::kidnap_callback(irobot_create_msgs::msg::HazardDetectionVector::SharedPtr msg)
{
  std::vector<irobot_create_msgs::msg::HazardDetection> hazard_vector = msg->detections;
 if(hazard_vector.size())
 {
   for(int i = 0; i < hazard_vector.size(); i++)
   {
     std::cout<<hazard_vector[i]<<std::endl;
   }
 }
}

void MockPublisher::stop_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto position = msg->pose.pose.position;
  auto orientation = msg->pose.pose.orientation;

  if(abs(position.x-pose_x)<0.00001 && abs(position.y-pose_y)<0.00001 && abs(position.z-pose_z)<0.00001 &&
     abs(orientation.x-orientation_x)<0.00001 && abs(orientation.y-orientation_y)<0.00001 &&
     abs(orientation.z-orientation_z)<0.00001 && abs(orientation.w-orientation_w)<0.00001)
  {
    stop_status_ = true;
  } else {
    stop_status_ = false;
  }

  // Store the current position
  pose_x = position.x;
  pose_y = position.y;
  pose_z = position.z;

  orientation_x = orientation.x;
  orientation_y = orientation.y;
  orientation_z = orientation.z;
  orientation_w = orientation.w;

  // Set header timestamp.
  this->stop_status_msg_.header.stamp = now();
  this->stop_status_msg_.is_stopped = stop_status_;

  // Publish topics
  this->stop_status_publisher_->publish(this->stop_status_msg_);
}

void MockPublisher::lightring_callback(irobot_create_msgs::msg::LightringLeds::SharedPtr msg)
{
  auto leds_vector = msg->leds;
}
}  // namespace irobot_create_toolbox
