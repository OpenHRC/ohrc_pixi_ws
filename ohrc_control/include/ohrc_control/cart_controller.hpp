#ifndef CART_CONTROLLER_HPP
#define CART_CONTROLLER_HPP

#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>
#include <boost/date_time.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <csignal>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <mutex>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "magic_enum.hpp"
#include "ohrc_common/utility.h"
#include "ohrc_control/arm_marker.hpp"
#include "ohrc_control/my_ik.hpp"
#include "ohrc_control/ohrc_control.hpp"
#include "ohrc_msgs/msg/state_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

// TODO: Add namespace "Controllers"?

using namespace ohrc_control;
using namespace std::placeholders;
using namespace std::chrono_literals;

class CartController : public rclcpp::Node {
  rclcpp::Node::SharedPtr node;

  double feedback_gain = 4.0;
  bool reseted = false;

  // rclcpp::CallbackGroup::SharedPtr

  std::string header;

  void init(std::string robot);
  void init(std::string robot, std::string hw_config);

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pubEefForce;

  std::mutex mtx_q;

  KDL::Frame des_eef_pose, current_eef_pose;
  KDL::Twist des_eef_vel;
  KDL::JntArray q_cur, dq_cur;
  std_msgs::msg::Float64MultiArray cmd;
  Matrix3d userManipU;
  int rc;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  struct s_cbJntState {
    bool isFirst = true;
    bool initialized = false;
  } s_cbJntState;

  struct s_moveInitPos {
    bool isFirst = true;
    KDL::JntArray q_des;
    KDL::JntArray q_initial;
    rclcpp::Time t_s;
    bool isSentTrj = false;
  } s_moveInitPos;

  struct s_initCmd {
    VectorXd q_des_t;
    VectorXd dq_des_t;
    KDL::JntArray q_cur;
    bool lastLoop;
    double T;
    double s;
    bool flag = false;
  } initCmd_;

  std::vector<double> _q_init_expect;

  std::string initPoseFrame;
  std::vector<double> initPose;
  bool getRosParams();

  std::string publisherTopicName;

  // ros::ServiceServer service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service;
  bool resetService(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response>& res);

  bool initialized = false;

  KDL::JntArray q_rest;

  rclcpp::Time prev_time;

  bool ftFound = false;
  const bool unique_state;

  Affine3d T_init;

  std::string ft_sensor_link, ft_topic;

protected:
  SolverType solver;
  ControllerType controller;
  PublisherType publisher;

  // ros::Subscriber jntStateSubscriber, userArmMarker, subForce;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subJntState;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subForce;
  // ros::TransportHints th = ros::TransportHints().tcpNoDelay(true);

  std::vector<bool*> subFlagPtrs;

  bool flagEffPose = false;

  Matrix3d _userManipU = Matrix3d::Identity();
  bool flagArmMarker = false;
  bool useManipOpt = false;

  int num_samples;

  double timeout;

  Affine3d Tft_eff;

  unsigned int nJnt;         // number of robot joint
  const unsigned int m = 6;  // number of target DoF (usually 6)

  std::vector<std::string> nameJnt;

  KDL::Frame _des_eef_pose;
  KDL::Frame _current_eef_pose;
  KDL::Twist _des_eef_vel;

  geometry_msgs::msg::WrenchStamped _force;
  bool flagForce = false;

  bool _disable = true, _passThrough = false;

  KDL::JntArray _q_cur, _dq_cur, _q_cur_t, _dq_cur_t;
  KDL::Frame _frame_cur;
  KDL::Twist _twist_cur;

  std::vector<butterworth> posFilter, velFilter, jntFilter;

  std::unique_ptr<TransformUtility> trans;

  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_ptr;
  KDL::Chain chain;
  std::vector<KDL::Segment> chain_segs;

  // MyIK
  std::shared_ptr<MyIK::MyIK> myik_solver_ptr;

  std::string root_frame;
  std::string chain_start, chain_end, urdf_param, robot_ns = "", hw_config_ns = "";
  Affine3d T_base_root;

  void initMembers();

  void cbJntState(const sensor_msgs::msg::JointState::SharedPtr msg);
  // void cbArmMarker(const visualization_msgs::msg::MarkerArray::SharedPtr& msg);
  void cbForce(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

  void initDesWithJnt(const KDL::JntArray& q_init);
  virtual void initWithJnt(const KDL::JntArray& q_init);
  // virtual void getDesEffPoseVel(const double& dt, const KDL::JntArray& q_cur, const KDL::JntArray& dq_cur,
  // KDL::Frame& des_eef_pose, KDL::Twist& des_eef_vel);
  void filterDesEffPoseVel(KDL::Frame& des_eef_pose, KDL::Twist& des_eef_vel);

  int moveInitPos(const KDL::JntArray& q_cur, const KDL::JntArray& dq_cur, const std::vector<std::string> nameJnt, std::vector<int> idxSegJnt);

  const int index = 0;

  void sendPositionCmd(const VectorXd& q_des);
  void sendJointStateCmd(const VectorXd& q_des, const VectorXd& dq_des);
  void sendVelocityCmd(const VectorXd& dq_des);
  void sendVelocityCmd(const VectorXd& q_des, const VectorXd& dq_des, const KDL::JntArray& q_cur, const bool& lastLoop);
  void sendTrajectoryCmd(const VectorXd& q_des, const double& T);
  void sendTrajectoryCmd(const VectorXd& q_des, const VectorXd& dq_des, const double& T);
  void sendTrajectoryActionCmd(const VectorXd& q_des, const double& T);
  void sendTrajectoryActionCmd(const VectorXd& q_des, const VectorXd& dq_des, const double& T);
  void getTrajectoryCmd(const VectorXd& q_des, const double& T, trajectory_msgs::msg::JointTrajectory& cmd_trj);
  void getTrajectoryCmd(const VectorXd& q_des, const VectorXd& dq_des, const double& T, trajectory_msgs::msg::JointTrajectory& cmd_trj);

  rclcpp::Time _lastJntStateUpdate;

public:
  float gripperCmd = 1.0;
  // CartController();
  CartController(rclcpp::Node::SharedPtr& node, const std::string robot, const std::string root_frame, const ControllerType controller, const double freq,
                 const bool unique_state = false);
  CartController(rclcpp::Node::SharedPtr& node, const std::string robot, const std::string root_frame, const int index, const ControllerType controller, const double freq,
                 const bool unique_state = false);
  CartController(rclcpp::Node::SharedPtr& node, const std::string robot, const std::string hw_config, const std::string root_frame, const int index,
                 const ControllerType controller, const double freq, const bool unique_state = false);
  // ~CartController();
  int control();

  rclcpp::SubscriptionOptions options;

  void sendIntJntCmd();
  void sendIntJntCmd(CartController::s_initCmd initCmd);
  void sendIntJntCmd(VectorXd q_des_t, VectorXd dq_des_t, KDL::JntArray q_cur, bool lastLoop, double T, double s);

  void update();
  void update(const rclcpp::Time& time, const rclcpp::Duration& period);
  void starting(const rclcpp::Time& time);
  void stopping(const rclcpp::Time& time);

  void initFt();

  void getIKInput(double dt, KDL::JntArray& q_cur, KDL::Frame& des_eef_pose, KDL::Twist& des_eef_vel);
  void getVelocity(const KDL::Frame& frame, const KDL::Frame& prev_frame, const double& dt, KDL::Twist& twist) const;

  inline void updateCurState() {
    std::lock_guard<std::mutex> lock(mtx_q);
    if ((node->get_clock()->now() - _lastJntStateUpdate).seconds() > 1.0) {
      RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), "No joint state update received for 1 second.");
      rclcpp::shutdown();
    }

    this->_q_cur_t = this->_q_cur;
    this->_dq_cur_t = this->_dq_cur;

    JntToCart(_q_cur_t, _dq_cur_t, _frame_cur, _twist_cur);
  }

  inline void getState(KDL::JntArray& q_cur, KDL::JntArray& dq_cur) {
    q_cur = this->_q_cur_t;
    dq_cur = this->_dq_cur_t;
  }

  inline void getState(KDL::JntArray& q_cur, KDL::JntArray& dq_cur, KDL::Frame& frame, KDL::Twist& twist) {
    getState(q_cur, dq_cur);
    getCartState(frame, twist);
  }

  inline void getCartState(KDL::Frame& frame, KDL::Twist& twist) {
    frame = this->_frame_cur;
    twist = this->_twist_cur;
  }

  void publishStates();
  void publishDesEffPoseVel(const KDL::Frame& des_eef_pose, const KDL::Twist& des_eef_vel);
  void publishCurEffPoseVel(const KDL::Frame& cur_eef_pose, const KDL::Twist& cur_eef_vel);
  void getDesState(const KDL::Frame& cur_pose, const KDL::Twist& cur_vel, KDL::Frame& des_pose, KDL::Twist& des_vel);
  void publishState(const KDL::Frame& pose, const KDL::Twist& vel, rclcpp::Publisher<ohrc_msgs::msg::State>::SharedPtr publisher);
  void publishState(const KDL::Frame& pose, const KDL::Twist& vel, const geometry_msgs::msg::Wrench& wrench, rclcpp::Publisher<ohrc_msgs::msg::State>::SharedPtr publisher);
  void publishMarker(const KDL::JntArray q_cur);

  void sendGripperCmd();

  void filterJnt(KDL::JntArray& q);
  void updatePosFilterCutoff(const double posFreq);
  void updateVelFilterCutoff(const double velFreq);
  void updateJntFilterCutoff(const double jntFreq);
  void updateFilterCutoff(const double velFreq, const double jntFreq);

  inline void startOperation() {
    _disable = false;
    _passThrough = true;
  }

  inline void enableOperation() {
    _disable = false;
  }

  inline void disableOperation() {
    _disable = true;
  }

  inline bool getOperationEnable() const {
    return !_disable;
  }
  inline void getInfo(std::string& chain_start, std::string& chain_end, Affine3d& T_base_root, std::shared_ptr<MyIK::MyIK>& myik) {
    chain_start = getChainStart();
    chain_end = this->chain_end;
    T_base_root = this->T_base_root;
    myik = this->myik_solver_ptr;
  }
  inline std::string getChainStart() const {
    return chain_start;
  }

  inline std::string getChainEnd() const {
    return chain_end;
  }

  inline void JntToCart(const KDL::JntArray& q_in, KDL::Frame& p_out) {
    myik_solver_ptr->JntToCart(q_in, p_out);
  }

  inline void JntToCart(const KDL::JntArray& q_in, const KDL::JntArray& dq_in, KDL::Frame& p_out, KDL::Twist& v_out) {
    myik_solver_ptr->JntToCart(q_in, dq_in, p_out, v_out);
  }

  inline void JntVelToCartVel(const KDL::JntArray& q_in, const KDL::JntArray& dq_in, KDL::Twist& v_out) {
    myik_solver_ptr->JntVelToCartVel(q_in, dq_in, v_out);
  }

  inline std::string getRobotNs() const {
    return robot_ns;
  }

  inline int getIndex() {
    return index;
  }
  Affine3d getTransform_base(std::string target);

  inline Affine3d getT_init() const {
    return T_init;
  }

  inline Affine3d getT_cur() {
    KDL::JntArray q_cur, dq_cur;
    getState(q_cur, dq_cur);

    KDL::Frame p;
    JntToCart(q_cur, p);

    Affine3d T;
    tf2::transformKDLToEigen(p, T);

    return T;
  }

  inline Affine3d getT_root() {
    return myik_solver_ptr->getT_base_world() * getT_cur();
  }

  inline Affine3d getT_base_world() {
    return myik_solver_ptr->getT_base_world();
  }

  inline geometry_msgs::msg::WrenchStamped getForceEef() {
    std::lock_guard<std::mutex> lock(mtx);
    return this->_force;
  }

  inline VectorXd getForceEefVec() {
    return tf2::fromMsg(getForceEef().wrench);
  }

  inline unsigned int getNJnt() {
    return nJnt;
  }

  inline std::vector<std::string> getNameJnt() {
    return nameJnt;
  }

  inline bool isInitialized() {
    return initialized;
  }

  inline void setDesired(const KDL::Frame& des_eef_pose, const KDL::Twist& des_eef_vel) {
    std::lock_guard<std::mutex> lock(mtx);
    this->_des_eef_pose = des_eef_pose;
    this->_des_eef_vel = des_eef_vel;
    // filterDesEffPoseVel(this->_des_eef_pose, this->_des_eef_vel);
  }

  inline void getDesired(KDL::Frame& des_eef_pose, KDL::Twist& des_eef_vel) {
    std::lock_guard<std::mutex> lock(mtx);
    des_eef_pose = this->_des_eef_pose;
    des_eef_vel = this->_des_eef_vel;
  }

  inline void enablePoseFeedback() {        // no longer used
    myik_solver_ptr->enablePoseFeedback();  // no longer used
  }

  inline void disablePoseFeedback() {        // no longer used
    myik_solver_ptr->disablePoseFeedback();  // no longer used
  }

  inline Affine3d getTft_eff() {
    return Tft_eff;
  }

  inline KDL::JntArray getqRest() {
    return q_rest;
  }

  inline void setqRest(const KDL::JntArray& q_rest) {
    // this->q_rest = q_rest;
  }

  inline bool getFtFound() {
    return ftFound;
  }

  rclcpp::Node::SharedPtr getNode() {
    return node;
  }

  void getInitCmd(VectorXd& q_des_t, VectorXd& dq_des_t, KDL::JntArray& q_cur, int& lastLoop, double& T, double& s) {
    q_des_t = initCmd_.q_des_t;
    dq_des_t = initCmd_.dq_des_t;
    q_cur = initCmd_.q_cur;
    lastLoop = int(initCmd_.lastLoop);
    T = initCmd_.T;
    s = initCmd_.s;
  }

  double dt;
  double freq = 500.0;

  const double eps = 1e-6;

  // ros::Publisher markerPublisher, desStatePublisher, curStatePublisher;

  // template <typename MsgType>
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jntCmdPublisher;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripperCmdPublisher;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jntStateCmdPublisher;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr jntTrjCmdPublisher;
  rclcpp::Publisher<ohrc_msgs::msg::State>::SharedPtr desStatePublisher, curStatePublisher;
  // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPublisher;

  std::mutex mtx;
  bool flagJntState = false;

  void resetPose();
  void resetFt();

  void sendPosCmd(const KDL::JntArray& q_des, const KDL::JntArray& dq_des, const double& dt);
  void sendVelCmd(const KDL::JntArray& q_des, const KDL::JntArray& dq_des, const double& dt);

  bool getReseted() {
    return this->reseted;
  }
  void setReseted(bool reseted) {
    this->reseted = reseted;
  }
};

#endif  // CART_CONTROLLER_HPP