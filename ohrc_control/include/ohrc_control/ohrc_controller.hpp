#ifndef OHRC_COTNROLLER_HPP
#define OHRC_COTNROLLER_HPP

#include <chrono>
#include <numeric>
#include <thread>

#include "ohrc_control/base_controllers.hpp"
#include "ohrc_control/cart_controller.hpp"
#include "ohrc_control/interface.hpp"
#include "ohrc_control/my_ik.hpp"
#include "ohrc_control/ohrc_control.hpp"
#include "ohrc_msgs/srv/set_priority.hpp"

using namespace std::placeholders;
using namespace ohrc_control;
using namespace std::chrono_literals;

class OhrcController : public rclcpp::Node {
  rclcpp::executors::MultiThreadedExecutor exec;
  std::vector<rclcpp::Node::SharedPtr> nodes;
  rclcpp::Node::SharedPtr node;

  bool isStarted = false, isControllerInitialized = false;

  std::vector<std::shared_ptr<MyIK::MyIK>> myik_ptr;

  rclcpp::TimerBase::SharedPtr control_timer;
  bool getRosParams(std::vector<std::string>& robots, std::vector<std::string>& hw_configs);
  void initMenbers(const std::vector<std::string> robots, const std::vector<std::string> hw_configs);
  void updateDesired();
  std::vector<KDL::Frame> desPose;
  std::vector<KDL::Twist> desVel;

  // ros::ServiceServer service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resetServer;
  rclcpp::Service<ohrc_msgs::srv::SetPriority>::SharedPtr priorityServer;

  void resetService(const std::shared_ptr<std_srvs::srv::Empty::Request> req, const std::shared_ptr<std_srvs::srv::Empty::Response>& res);
  void priorityService(const std::shared_ptr<ohrc_msgs::srv::SetPriority::Request> req, const std::shared_ptr<ohrc_msgs::srv::SetPriority::Response>& res);

  void publishState(const rclcpp::Time& time, const std::vector<KDL::Frame> curPose, const std::vector<KDL::Twist> curVel, const std::vector<KDL::Frame> desPose,
                    const std::vector<KDL::Twist> desVel);

  void controlLoop();
  virtual void starting();
  void stopping();
  void update(const rclcpp::Time& time, const rclcpp::Duration& period);

  bool initRobotPose();
  void initController();

  enum class MFMode { Individual, Parallel, Cooperation, None } MFmode;
  enum class IKMode { Concatenated, Order, Parallel, None } IKmode;

  // std::vector<std::string> robots;
  // std::vector<std::string> hw_configs;
  int nRobot = 0;
  bool unique_state = false;

  std::string root_frame;
  double freq = 500.0;
  double dt;
  rclcpp::Time t0;
  std::string date;

  // MyIK
  std::unique_ptr<MyIK::MyIK> multimyik_solver_ptr;

  ControllerType controller;
  PublisherType publisher;

  std::vector<int> manualInd, autoInd;

  std::vector<rclcpp::Time> prev_time;

  enum class PriorityType { Manual, Automation, Adaptation, None } priority;
  bool adaptation = false;
  void setPriority(PriorityType priority);
  void setPriority(std::vector<int> idx);
  void setPriority(int i);
  void setLowPriority(int i);
  void setHightLowPriority(int high, int low);

  // enum class AdaptationOption { Default, None } adaptationOption;
  std::string adaptationOption_;

  // FeedbackMode feedbackMode;

  virtual void runLoopEnd() {};

  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist, Interfaces& interfaces_);
  void updateAllCurState();

  void initInterface(const std::vector<std::shared_ptr<Interface>> interfaces_);
  void resetInterface(const std::vector<std::shared_ptr<Interface>> interfaces_);
  void feedback(KDL::Frame& pose, KDL::Twist& twist, const std::vector<std::shared_ptr<Interface>> interfaces_);

  std::vector<bool> _isEnable;
  void selectInterface(std::vector<bool> isEnable);

  // virtual void preInterfaceProcess(std::vector<std::shared_ptr<Interface>> interfaces) {};

  // virtual void updateManualTargetPose(KDL::Frame& pose, KDL::Twist& twist, const std::shared_ptr<CartController>& controller) {
  //   updateTargetPose(pose, twist, controller);
  // };
  // virtual void updateAutoTargetPose(KDL::Frame& pose, KDL::Twist& twist, const std::shared_ptr<CartController>& controller) {
  //   updateTargetPose(pose, twist, controller);
  // };
  // virtual void feedbackJnt(const KDL::JntArray& q_cur, const KDL::JntArray& q_des, std::shared_ptr<CartController> controller){};
  // virtual void feedbackCart(const Affine3d& T_cur, const Affine3d& T_des, std::shared_ptr<CartController> controller){};

  int interfaceIdx = -1;

protected:
  virtual void defineInterface() {};
  virtual void initControllerAdditional() {};

  std::vector<bool> priorityIdx;

  std::vector<Interfaces> interfaces;
  std::vector<std::shared_ptr<CartController>> cartControllers;

  virtual void overrideDesired(std::vector<KDL::Frame>& desPose, std::vector<KDL::Twist>& desVel) {};

  // virtual std::shared_ptr<Interface> selectInterface(std::shared_ptr<CartController> cartController) {
  // }

  int getNRobot() {
    return nRobot;
  }

public:
  OhrcController();
  ~OhrcController();
  void control();

  rclcpp::executors::MultiThreadedExecutor::SharedPtr getExecutor() {
    return exec.make_shared();
  }
};

#endif  // OHRC_COTNROLLER_HPP
