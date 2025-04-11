#include "ohrc_teleoperation/pose_topic_interface.hpp"

void PoseTopicInterface::initInterface() {
  setSubscriber();

  Affine3d T_state_base = controller->getTransform_base(this->stateFrameId);
  R = T_state_base.rotation();

  bool diablePoseFeedback;
  RclcppUtility::declare_and_get_parameter(node, "diable_pose_feedback", false, diablePoseFeedback);

  controller->updatePosFilterCutoff(10.0);
  controller->enableOperation();
}

void PoseTopicInterface::setSubscriber() {
  getTopicAndFrameName("cmd_pose", "user_frame");

  subPose =
      node->create_subscription<geometry_msgs::msg::Pose>(stateTopicName, rclcpp::QoS(1), std::bind(&PoseTopicInterface::cbPose, this, std::placeholders::_1), controller->options);
}

void PoseTopicInterface::cbPose(const geometry_msgs::msg::Pose::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx);
  _pose = *msg;
  _flagTopic = true;

  updateIsEnable(true);
}

void PoseTopicInterface::updateTargetPose(const rclcpp::Time t, KDL::Frame& pose, KDL::Twist& twist) {
  geometry_msgs::msg::Pose markerPose;
  double markerDt;
  {
    std::lock_guard<std::mutex> lock(mtx);
    if (!_flagTopic) {
      return;
    }
    markerPose = _pose;
  }

  Affine3d T_topic;
  tf2::fromMsg(markerPose, T_topic);
  Affine3d T_base = Translation3d(R * T_topic.translation()) * (R * T_topic.rotation());

  static Affine3d initT = T_base;

  Vector3d pos = T_base.translation() - initT.translation() + controller->getT_init().translation();

  tf2::vectorEigenToKDL(pos, pose.p);
}

void PoseTopicInterface::resetInterface() {
  // RCLCPP_INFO_STREAM(node->get_logger(), "Reset pose position");
  _flagTopic = false;
}