#ifndef JOY_TOPIC_ITNERFACE_HPP
#define JOY_TOPIC_ITNERFACE_HPP

#include "ohrc_teleoperation/twist_topic_interface.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoyTopicInterface : virtual public TwistTopicInterface {
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subJoy;
  sensor_msgs::msg::Joy _joy;

  double gain_h = 0.1, gain_r = 0.1;

protected:
  // std::string stateTopicName = "/spacenav/joy", stateFrameId = "world";

  void cbJoy(const sensor_msgs::msg::Joy::SharedPtr msg);
  void setSubscriber() override;

public:
  using TwistTopicInterface::TwistTopicInterface;
  void updateTargetPose(const rclcpp::Time t, KDL::Frame& pose, KDL::Twist& twist) override;
  void initInterface() override;
  // void resetInterface() override;
};

#endif  // JOY_TOPIC_ITNERFACE_HPP