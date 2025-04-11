#include "ohrc_teleoperation/xr_body_interface.hpp"

void XrBodyInterface::initInterface() {
  setSubscriber();
  T_state_base = controller->getTransform_base(this->stateFrameId);

  RclcppUtility::declare_and_get_parameter_enum(node, robot_ns + "body_part", BodyPart::RIGHT_HAND, bodyPart);

  // controller->updateFilterCutoff(10.0, 10.0);
  controller->disablePoseFeedback();
  controller->updateFilterCutoff(20.0, 20.0);

  pubFeedback = node->create_publisher<std_msgs::msg::Float32>(std::string("/feedback/") + std::string(magic_enum::enum_name(bodyPart)), rclcpp::QoS(2));
}

void XrBodyInterface::setSubscriber() {
  getTopicAndFrameName("/body_state", "user_frame");

  subBody = node->create_subscription<ohrc_msgs::msg::BodyState>(stateTopicName, rclcpp::QoS(1), std::bind(&XrBodyInterface::cbBody, this, std::placeholders::_1));
}

void XrBodyInterface::cbBody(const ohrc_msgs::msg::BodyState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx);

  ohrc_msgs::msg::BodyState body = *msg;
  ohrc_msgs::msg::State state = this->_state;
  state.enabled = false;
  switch (bodyPart) {
    case BodyPart::RIGHT_HAND:
      state.pose = body.right_hand.pose;
      state.twist = body.right_hand.twist;
      state.reset = body.right_hand.button[2];  // stick click
      if (getEnableFlag(body.right_hand, body.left_hand))
        state.enabled = true;
      break;

    case BodyPart::LEFT_HAND:
      state.pose = body.left_hand.pose;
      state.twist = body.left_hand.twist;
      state.reset = body.left_hand.button[2];  // stick click
      if (getEnableFlag(body.left_hand, body.right_hand))
        state.enabled = true;
      break;

    case BodyPart::HEAD:
      state.pose = body.head.pose;
      state.twist = body.head.twist;
      if (getEnableFlag(body.right_hand, body.left_hand) || getEnableFlag(body.left_hand, body.right_hand))
        state.enabled = true;
      break;

    case BodyPart::EITHER_HANDS:
      if (getEnableFlag(body.right_hand, body.left_hand)) {
        hand = Hand::RIGHT;
        state.enabled = true;
      } else if (getEnableFlag(body.left_hand, body.right_hand)) {
        hand = Hand::LEFT;
        state.enabled = true;
      }

      if (hand == Hand::RIGHT) {
        state.pose = body.right_hand.pose;
        state.twist = body.right_hand.twist;
        state.reset = body.right_hand.button[2];  // stick click
      } else {
        state.pose = body.left_hand.pose;
        state.twist = body.left_hand.twist;
        state.reset = body.left_hand.button[2];  // stick click
      }
      break;

    default:
      break;
  }

  if (state.reset)
    this->reset();

  isEnable = state.enabled;

  this->_state = state;
  _flagTopic = true;
}

bool XrBodyInterface::getEnableFlag(const ohrc_msgs::msg::BodyPartState& handState, const ohrc_msgs::msg::BodyPartState& anotherBodyPartState) {
  if (handState.grip > 0.95)
    return true;
  else
    return false;
}

void XrBodyInterface::updateTargetPose(const rclcpp::Time t, KDL::Frame& pose, KDL::Twist& twist) {
  ohrc_msgs::msg::State state;
  {
    std::lock_guard<std::mutex> lock(mtx);
    state = this->_state;
    if (!_flagTopic)
      return;
  }

  // ROS_INFO_STREAM(state);
  getTargetState(state, pose, twist);
  this->_state = state;  // TODO: check if this is necessary
}

// void XrBodyInterface::resetInterface() {
//   this->isFirst = true;
// }

void XrBodyInterface::feedback(const KDL::Frame& targetPos, const KDL::Twist& targetTwist) {
  std_msgs::msg::Float32 amp;

  amp.data = std::max(std::min((tf2::fromMsg(controller->getForceEef().wrench).head(3).norm() - 1.0) / 10.0, 1.0), 0.0);

  // if (controller->getOperationEnable())
  // pubFeedback->publish(amp);
  // else
  //   pubFeedback->publish(std_msgs::msg::Float32());
}