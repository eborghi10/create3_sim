// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Lola Segura (lsegura@irobot.com)

#ifndef IROBOT_CREATE_TOOLBOX__MOCK_PUBLISHER_HPP_
#define IROBOT_CREATE_TOOLBOX__MOCK_PUBLISHER_HPP_

#include <irobot_create_msgs/action/led_animation.hpp>
#include <irobot_create_msgs/msg/button.hpp>
#include <irobot_create_msgs/msg/interface_buttons.hpp>
#include <irobot_create_msgs/msg/led_color.hpp>
#include <irobot_create_msgs/msg/lightring_leds.hpp>
#include <irobot_create_msgs/msg/slip_status.hpp>
#include <irobot_create_toolbox/parameter_helper.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <limits>
#include <memory>
#include <string>
#include <vector>


namespace irobot_create_toolbox
{
class MockPublisher : public rclcpp::Node
{
public:
  /// \brief Constructor
  MockPublisher();

  // Callback functions
  void lightring_callback(irobot_create_msgs::msg::LightringLeds::SharedPtr msg);

protected:
  rclcpp_action::GoalResponse handle_led_animation_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const irobot_create_msgs::action::LedAnimation::Goal> goal);
  rclcpp_action::CancelResponse handle_led_animation_cancel(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::LedAnimation>> goal_handle);
  void handle_led_animation_accepted(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::LedAnimation>> goal_handle);
  void execute_led_animation(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::LedAnimation>> goal_handle);

  // Publish aggregated detections on timer_'s frequency
  rclcpp::TimerBase::SharedPtr buttons_timer_;
  rclcpp::TimerBase::SharedPtr slip_status_timer_;

  // Publishers
  std::shared_ptr<
    rclcpp::Publisher<irobot_create_msgs::msg::InterfaceButtons>> buttons_publisher_{nullptr};
  rclcpp::Publisher<irobot_create_msgs::msg::SlipStatus>::SharedPtr slip_status_publisher_{nullptr};

  // Subscribers
  rclcpp::Subscription<irobot_create_msgs::msg::LightringLeds>::SharedPtr lightring_subscription_;

  // Actions
  rclcpp_action::Server<irobot_create_msgs::action::LedAnimation>::SharedPtr
    led_animation_action_server_;

  // Gazebo simulator being used
  std::string gazebo_;

  // Topic to publish interface buttons to
  std::string buttons_publisher_topic_;
  // Topic to publish slip status to
  std::string slip_status_publisher_topic_;

  // Topic to subscribe to light ring vector
  std::string lightring_subscription_topic_;

  // Message to store the interface buttons
  irobot_create_msgs::msg::InterfaceButtons buttons_msg_;
  // Message to store the slip status
  irobot_create_msgs::msg::SlipStatus slip_status_msg_;

  const std::string base_frame_ {"base_link"};
  std::mutex led_animation_params_mutex_;
  rclcpp::Duration led_animation_end_duration_;
  rclcpp::Time led_animation_start_time_;
};

}  // namespace irobot_create_toolbox

#endif  // IROBOT_CREATE_TOOLBOX__MOCK_PUBLISHER_HPP_
