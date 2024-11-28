#ifndef INTERFACE_HPP
#define INTERFACE_HPP

#include "ohrc_control/cart_controller.hpp"

class Interface {
protected:
  // ros::NodeHandle n;
  const double dt;
  rclcpp::Node::SharedPtr node;

  // std::shared_ptr<TransformUtility> trans;

  std::shared_ptr<CartController> controller;

  // ros::TransportHints th = ros::TransportHints().tcpNoDelay(true);

  TaskState taskState = TaskState::Initial;

  std::string targetName;
  double targetDistance = 0.0;
  VectorXd e;

  inline void reset() {
    controller->resetPose();
    controller->resetFt();
    resetInterface();
  }

  std::string stateTopicName = "/state", stateFrameId = "world";
  inline void getTopicAndFrameName(std::string DefaultStateTopicName, std::string DefaultStateFrameId) {
    RclcppUtility::declare_and_get_parameter(node, "topic_name", DefaultStateTopicName, this->stateTopicName);
    RclcppUtility::declare_and_get_parameter(node, "frame_id", DefaultStateFrameId, this->stateFrameId);

    if (stateTopicName[0] != '/')
      stateTopicName = controller->getRobotNs() + stateTopicName;
  }

public:
  Interface(const std::shared_ptr<CartController>& controller) : node(controller->getNode()), dt(controller->dt) {
    this->controller = controller;
    // trans = std::make_shared<TransformUtility>(node);
  }

  virtual void updateTargetPose(const rclcpp::Time t, KDL::Frame& pose, KDL::Twist& twist) {};
  virtual void initInterface() {};
  virtual void resetInterface() {};
  virtual void feedback(const KDL::Frame& targetPos, const KDL::Twist& targetTwist) {};

  int targetIdx = -1, nCompletedTask = 0;
  bool blocked = false;

  inline std::string getTargetName() {
    return this->targetName;
  }

  inline double getTargetDistance() {
    return this->targetDistance;
  }

  inline TaskState getTaskState() {
    return this->taskState;
  }

  inline VectorXd getTargetError() {
    return this->e;
  }
};

#endif  // INTERFACE_HPP