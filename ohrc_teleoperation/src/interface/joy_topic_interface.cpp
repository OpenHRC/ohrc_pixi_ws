#include "ohrc_teleoperation/joy_topic_interface.hpp"

void JoyTopicInterface::initInterface() {
  RclcppUtility::declare_and_get_parameter(node, "gain/horizontal", 1.0, gain_h);
  RclcppUtility::declare_and_get_parameter(node, "gain/rotational", 1.0, gain_r);

  TwistTopicInterface::initInterface();
}

void JoyTopicInterface::setSubscriber() {
  getTopicAndFrameName("/spacenav/joy", "user_frame");
  subJoy = node->create_subscription<sensor_msgs::msg::Joy>(stateTopicName, rclcpp::QoS(1), std::bind(&JoyTopicInterface::cbJoy, this, std::placeholders::_1));
}

void JoyTopicInterface::cbJoy(const sensor_msgs::msg::Joy::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_topic);
  _joy = *msg;
  _flagTopic = true;
}

void JoyTopicInterface::updateTargetPose(const rclcpp::Time t, KDL::Frame& pos, KDL::Twist& twist) {
  sensor_msgs::msg::Joy joy;
  {
    std::lock_guard<std::mutex> lock(mtx_topic);
    joy = _joy;
    if (!_flagTopic)
      return;
  }

  if (joy.axes.size() < 6 || joy.buttons.size() < 2)
    return;

  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = joy.axes[0] * gain_h;
  twist_msg.linear.y = joy.axes[1] * gain_h;
  twist_msg.linear.z = joy.axes[2] * gain_h;
  twist_msg.angular.x = joy.axes[3] * gain_r;
  twist_msg.angular.y = joy.axes[4] * gain_r;
  twist_msg.angular.z = joy.axes[5] * gain_r;

  if (joy.buttons[1] == 1.0)
    this->reset();

  KDL::Frame pos_joy = pos;
  KDL::Twist twist_joy = twist;
  setPoseFromTwistMsg(twist_msg, pos, twist);

  // twist = twist_joy + twist;
}

// void JoyTopicInterface::resetInterface() {
//   ROS_WARN_STREAM("Reset marker position");
//   state = ohrc_msgs::State();
// }