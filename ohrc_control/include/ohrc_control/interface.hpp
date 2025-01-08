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

    if (controller->getFtFound())
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

  bool isEnable = false;

  std::string inaterfaceName = "";
  bool updateIsEnable(bool condition) {
    if (condition && !isEnable) {
      RCLCPP_INFO_STREAM(node->get_logger(), "[" + inaterfaceName + "] Enabled");
      resetInterface();
    } else if (!condition && isEnable)
      RCLCPP_INFO_STREAM(node->get_logger(), "[" + inaterfaceName + "] Disabled");

    isEnable = condition;
    return condition;
  }

public:
  Interface(const std::shared_ptr<CartController>& controller) : node(controller->getNode()), dt(controller->dt) {
    this->controller = controller;
    // trans = std::make_shared<TransformUtility>(node);
  }

  std::mutex mtx;

  virtual void updateTargetPose(const rclcpp::Time t, KDL::Frame& pose, KDL::Twist& twist) {};
  virtual void initInterface() {};
  virtual void resetInterface() {};
  virtual void updateInterface() {};
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

  inline bool getIsEnable() {
    return this->isEnable;
  }

  inline void setIsEnable(bool isEnable) {
    this->isEnable = isEnable;
  }
};

class Interfaces {
public:
  std::vector<std::shared_ptr<Interface>> interfaces;
  std::vector<bool> isEnables;

  void updateIsEnables() {
    for (size_t i = 0; i < interfaces.size(); i++) {
      std::lock_guard<std::mutex> lock(interfaces[i]->mtx);
      isEnables[i] = interfaces[i]->getIsEnable();
    }
  }
};

#endif  // INTERFACE_HPP