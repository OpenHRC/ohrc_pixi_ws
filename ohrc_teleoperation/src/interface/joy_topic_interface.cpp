#include "ohrc_teleoperation/joy_topic_interface.hpp"

void JoyTopicInterface::initInterface() {
  interfaceName = "JoyTopicInterface";

  RclcppUtility::declare_and_get_parameter_enum(this->node, interfaceName + ".feedback_mode", FeedbackMode::NoFeedback, feedbackMode);
  RclcppUtility::declare_and_get_parameter(node, interfaceName + ".gain.horizontal", 1.0, gain_h);
  RclcppUtility::declare_and_get_parameter(node, interfaceName + ".gain.rotational", 1.0, gain_r);

  // TwistTopicInterface::initInterface();

  setSubscriber();

  Affine3d T_state_base = controller->getTransform_base(this->stateFrameId);
  R = T_state_base.rotation().transpose();
}

void JoyTopicInterface::setSubscriber() {
  getTopicAndFrameName("/spacenav/joy", "user_frame");
  subJoy = node->create_subscription<sensor_msgs::msg::Joy>(stateTopicName, rclcpp::QoS(1), std::bind(&JoyTopicInterface::cbJoy, this, std::placeholders::_1), controller->options);
}

void JoyTopicInterface::cbJoy(const sensor_msgs::msg::Joy::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx);
  updateIsEnable(Eigen::Map<Eigen::VectorXf>(msg->axes.data(), msg->axes.size()).norm() > 1.0e-2 ||
                 std::any_of(msg->buttons.begin(), msg->buttons.end(), [](int i) { return i == 1; }));

  _joy = *msg;
  _flagTopic = true;
}

void JoyTopicInterface::updateTargetPose(const rclcpp::Time t, KDL::Frame& pos, KDL::Twist& twist) {
  sensor_msgs::msg::Joy joy;
  {
    std::lock_guard<std::mutex> lock(mtx);
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

  setPoseFromTwistMsg(twist_msg, pos, twist);
}

// void JoyTopicInterface::resetInterface() {
//   ROS_WARN_STREAM("Reset marker position");
//   state = ohrc_msgs::State();
// }