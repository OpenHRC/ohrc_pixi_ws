#include "ohrc_control/ohrc_controller.hpp"

OhrcController::OhrcController() : Node("ohrc_controller") {
}

OhrcController::~OhrcController() {
  this->stopping();
}

bool OhrcController::getRosParams(std::vector<std::string>& robots, std::vector<std::string>& hw_configs) {
  std::vector<std::string> _hw_configs;
  if (!RclcppUtility::declare_and_get_parameter(this->node, "follower.hw", std::vector<std::string>(), _hw_configs, false) || _hw_configs.empty() ||
      std::all_of(_hw_configs.begin(), _hw_configs.end(), [](std::string x) { return x == ""; })) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Failed to get the follower robot hw list");
    return false;
  }

  for (std::string hw_config : _hw_configs) {
    std::vector<std::string> _robots;

    if (!RclcppUtility::declare_and_get_parameter(this->node, "follower.ns." + hw_config, std::vector<std::string>(), _robots, false) || _robots.empty()) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Failed to get the follower robot ns list");
      return false;
    }

    for (auto robot : _robots) {
      robots.push_back(robot);
      hw_configs.push_back(hw_config);
    }
  }

  for (size_t i = 0; i < robots.size(); i++)
    RCLCPP_INFO_STREAM(this->get_logger(), "Configured Robot " << i << ": " << hw_configs[i] << " (ns: " << robots[i] << ")");

  if (!RclcppUtility::declare_and_get_parameter(this->node, "root_frame", std::string(""), root_frame))
    return false;

  if (!RclcppUtility::declare_and_get_parameter(this->node, "control_freq", double(0.0), freq))
    return false;

  if (!RclcppUtility::declare_and_get_parameter(this->node, "unique_state_pub_sub", false, unique_state))
    return false;

  if (!RclcppUtility::declare_and_get_parameter_enum(this->node, "controller", ControllerType::Velocity, controller))
    return false;

  // if (!RclcppUtility::declare_and_get_parameter_enum(node, "publisher", PublisherType::Velocity, publisher))
  //   return false;

  // if (!RclcppUtility::declare_and_get_parameter_enum(this->node, "feedback_mode", FeedbackMode::PositionFeedback, feedbackMode))
  //   return false;

  if (!RclcppUtility::declare_and_get_parameter_enum(this->node, "MF_mode", MFMode::Individual, MFmode))
    return false;

  if (!RclcppUtility::declare_and_get_parameter_enum(this->node, "IK_mode", IKMode::Concatenated, IKmode))
    return false;

  if (!RclcppUtility::declare_and_get_parameter_enum(this->node, "priority", PriorityType::Manual, priority))
    return false;

  // if (priority == PriorityType::Adaptation)
  //   n.param<std::string>("adaptation_option", adaptationOption_, "Default");

  // if (!n.getParam("date", this->date)) {
  //   this->date = std_utility::getDatetimeStr();
  //   n.setParam("date", date);
  // }

  return true;
}

void OhrcController::initMenbers(const std::vector<std::string> robots, const std::vector<std::string> hw_configs) {
  nRobot = robots.size();
  this->dt = 1.0 / this->freq;

  prev_time.resize(nRobot, this->get_clock()->now());
  cartControllers.resize(nRobot);
  nodes.resize(nRobot);

  for (size_t i = 0; i < nRobot; i++) {
    cartControllers[i] =
        std::make_shared<CartController>(nodes[i], robots[i], hw_configs[i], root_frame, i, controller, freq, unique_state);  // TODO: might can be replaced with sub_node
    exec.add_node(nodes[i]);
  }

  std::vector<std::string> base_link(nRobot), tip_link(nRobot);  // TODO: base_links for all robot shoud be same
  std::vector<Affine3d> T_base_root(nRobot);
  myik_ptr.resize(nRobot);
  for (size_t i = 0; i < nRobot; i++)
    cartControllers[i]->getInfo(base_link[i], tip_link[i], T_base_root[i], myik_ptr[i]);

  multimyik_solver_ptr = std::make_unique<MyIK::MyIK>(node, base_link, tip_link, T_base_root, myik_ptr);

  service = this->create_service<std_srvs::srv::Empty>("/reset", std::bind(&OhrcController::resetService, this, _1, _2));

  for (size_t i = 0; i < nRobot; i = i + 2)
    manualInd.push_back(i);

  for (int i = 1; i < nRobot; i = i + 2)
    autoInd.push_back(i);

  setPriority(priority);

  desPose.resize(nRobot);
  desVel.resize(nRobot);

  interfaces.resize(nRobot);

  this->defineInterface();

  // for (size_t i = 0; i < nRobot; i++) {
  //   // interfaces[i].push_back(selectInterface(cartControllers[i]));
  // }

  for (size_t i = 0; i < nRobot; i++) {
    initInterface(interfaces[i].interfaces);
    int nInterface = interfaces[i].interfaces.size();
    for (size_t j = 0; j < nInterface; j++) {
      interfaces[i].interfaces.push_back(ohrc_control::selectBaseController(interfaces[i].interfaces[j]->getFeedbackMode(), cartControllers[i]));
    }
    interfaces[i].isEnables.resize(interfaces[i].interfaces.size(), false);
    cartControllers[i]->disablePoseFeedback();  // TODO: Pose feedback would be always enable. original feedback code can be removed.
  }

  control_timer = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&OhrcController::controlLoop, this));
  // initilize_timer = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&OhrcController::initLoop, this));
  // initilize_timer->cancel();

  exec.add_node(this->node);
}

void OhrcController::selectInterface(std::vector<bool> isEnable) {
  if (std::all_of(isEnable.begin(), isEnable.end(), [](bool x) { return x == false; }))
    _isEnable = isEnable;
}

void OhrcController::updateTargetPose(KDL::Frame& pose, KDL::Twist& twist, Interfaces& interfaces_) {
  interfaces_.updateIsEnables();

  if (this->interfaceIdx == -1) {
    for (size_t i = 0; i < interfaces_.isEnables.size() - 1; i++)
      if (interfaces_.isEnables[i]) {
        this->interfaceIdx = i;
        break;
      }
  } else {
    if (!interfaces_.isEnables[this->interfaceIdx]) {
      this->interfaceIdx = -1;
      for (size_t i = 0; i < interfaces_.isEnables.size() - 1; i++)
        if (interfaces_.isEnables[i]) {
          this->interfaceIdx = i;
          break;
        }
    }
  }

  for (auto& interface : interfaces_.interfaces)
    interface->updateInterface();

  if (this->interfaceIdx != -1) {
    // apply the selected interface operation
    interfaces_.interfaces[this->interfaceIdx]->updateTargetPose(this->get_clock()->now(), pose, twist);

    // apply the base controller
    interfaces_.interfaces[interfaces_.interfaces.size() / 2 + interfaceIdx]->updateTargetPose(this->get_clock()->now(), pose, twist);
  }
}

// virtual void defineInterface() = 0;

void OhrcController::updateAllCurState() {
  for (auto cartController : cartControllers)
    cartController->updateCurState();
}

void OhrcController::initInterface(const std::vector<std::shared_ptr<Interface>> interfaces_) {
  for (auto& interface : interfaces_)
    interface->initInterface();

  _isEnable.resize(interfaces_.size() - 1);
}

void OhrcController::resetInterface(const std::vector<std::shared_ptr<Interface>> interfaces_) {
  for (auto& interface : interfaces_)
    interface->resetInterface();
}

void OhrcController::feedback(KDL::Frame& pose, KDL::Twist& twist, const std::vector<std::shared_ptr<Interface>> interfaces_) {
  for (auto& interface : interfaces_)
    interface->feedback(pose, twist);
}

void OhrcController::resetService(const std::shared_ptr<std_srvs::srv::Empty::Request> req, const std::shared_ptr<std_srvs::srv::Empty::Response>& res) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Resetting...");
  for (auto cartController : cartControllers)
    cartController->resetPose();

  isControllerInitialized = false;

  // while (!isControllerInitialized && rclcpp::ok()) {
  // rclcpp::sleep_for(std::chrono::milliseconds(100));
  // }
}

void OhrcController::setPriority(int i) {
  multimyik_solver_ptr->resetRobotWeight();  // make all robot priority equal.
  multimyik_solver_ptr->setRobotWeight(i, 100.);
}

void OhrcController::setLowPriority(int i) {
  multimyik_solver_ptr->resetRobotWeight();  // make all robot priority equal.
  // for (int j = 0; j++; j < nRobot)
  //   if (j != i)
  //     multimyik_solver_ptr->setRobotWeight(j, 100.);
  multimyik_solver_ptr->setRobotWeight(i, 0.1);
}

void OhrcController::setPriority(std::vector<int> idx) {
  multimyik_solver_ptr->resetRobotWeight();  // make all robot priority equal.

  double gain = pow(10.0, idx.size() - 1);
  for (auto& i : idx) {
    multimyik_solver_ptr->setRobotWeight(i, 100. * gain);
    gain *= 0.1;
  }
}

void OhrcController::setPriority(PriorityType priority) {
  std::vector<int> priorityInd;
  if (priority == PriorityType::Automation)
    priorityInd = autoInd;
  else
    priorityInd = manualInd;

  multimyik_solver_ptr->resetRobotWeight();  // make all robot priority equal.
  for (auto& ind : priorityInd)
    multimyik_solver_ptr->setRobotWeight(ind, 100.);
}

void OhrcController::setHightLowPriority(int high, int low) {
  multimyik_solver_ptr->resetRobotWeight();  // make all robot priority equal.
  multimyik_solver_ptr->setRobotWeight(high, 100.);
  multimyik_solver_ptr->setRobotWeight(low, 0.1);
}

void OhrcController::starting() {
  for (auto cartController : cartControllers)
    cartController->starting(this->get_clock()->now());
}

void OhrcController::initController() {
  updateAllCurState();
  for (auto cartController : cartControllers) {
    cartController->initFt();
    resetInterface(interfaces[cartController->getIndex()].interfaces);
  }

  std::vector<KDL::JntArray> q_rest(nRobot);
  for (size_t i = 0; i < nRobot; i++)
    q_rest[i] = cartControllers[i]->getqRest();
  multimyik_solver_ptr->setqRest(q_rest);

  // cartControllers[i]->enableOperation();
  // }

  RCLCPP_INFO_STREAM(this->get_logger(), "Controller started!");
  this->t0 = this->get_clock()->now();
}

void OhrcController::stopping() {
  for (auto cartController : cartControllers)
    cartController->stopping(this->get_clock()->now());  // TODO: Make sure that this works correctly.

  RCLCPP_INFO_STREAM(this->get_logger(), "Controller stopped!");

  rclcpp::shutdown();
}

void OhrcController::publishState(const rclcpp::Time& time, const std::vector<KDL::Frame> curPose, const std::vector<KDL::Twist> curVel, const std::vector<KDL::Frame> desPose,
                                  const std::vector<KDL::Twist> desVel) {
  static rclcpp::Time prev = time;
  if ((time - prev).nanoseconds() * 1.0e-9 > 0.05) {
    for (size_t i = 0; i < nRobot; i++) {
      cartControllers[i]->publishDesEffPoseVel(desPose[i], desVel[i]);
      cartControllers[i]->publishCurEffPoseVel(curPose[i], curVel[i]);
    }
    prev = time;
  }
}

void OhrcController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  static std::vector<KDL::JntArray> q_des(nRobot), dq_des(nRobot), q_cur(nRobot), dq_cur(nRobot);
  std::vector<KDL::Frame> curPose(nRobot);
  std::vector<KDL::Twist> curVel(nRobot);

  for (size_t i = 0; i < nRobot; i++)
    cartControllers[i]->getState(q_cur[i], dq_cur[i], curPose[i], curVel[i]);

  this->publishState(time, curPose, curVel, desPose, desVel);

  std::vector<KDL::JntArray> q_rest(nRobot);
  for (size_t i = 0; i < nRobot; i++)
    q_rest[i] = cartControllers[i]->getqRest();
  multimyik_solver_ptr->setqRest(q_rest);

  if (controller == ControllerType::Velocity) {
    int rc = multimyik_solver_ptr->CartToJntVel_qp(q_cur, desPose, desVel, dq_des, dt);

    if (rc < 0) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Failed to solve IK within dt. Skip this control loop");
      return;
    }

    // low pass filter
    for (size_t i = 0; i < nRobot; i++) {
      cartControllers[i]->filterJnt(dq_des[i]);
      if (q_des[i].data.rows() != dq_des[i].data.rows())
        q_des[i].data = cartControllers[i]->getqRest().data;
      q_des[i].data += dq_des[i].data * dt;
    }

    if (!unique_state) {
      for (size_t i = 0; i < nRobot; i++) {
        if ((time - prev_time[i]).nanoseconds() * 1.0e-9 < 1.0 / cartControllers[i]->freq - 1.0 / this->freq)
          continue;

        prev_time[i] = time;
        // std::cout << q_des[i].data.transpose() << std::endl;
        // std::cout << dq_des[i].data.transpose() << std::endl;
        cartControllers[i]->sendVelCmd(q_des[i], dq_des[i], 1.0 / cartControllers[i]->freq);
      }
    } else {
      KDL::JntArray q_des_all, dq_des_all;
      q_des_all.data = math_utility::concatenateVecOfVec(q_des);
      dq_des_all.data = math_utility::concatenateVecOfVec(dq_des);

      cartControllers[0]->sendVelCmd(q_des_all, dq_des_all, 1.0 / cartControllers[0]->freq);
    }

  } else if (controller == ControllerType::Position) {
    int rc = multimyik_solver_ptr->CartToJnt(q_cur, desPose, q_des, dt);

    if (rc < 0) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Failed to solve IK within dt. Skip this control loop");
      return;
    }

    // low pass filter
    for (size_t i = 0; i < nRobot; i++)
      cartControllers[i]->filterJnt(q_des[i]);

    for (size_t i = 0; i < nRobot; i++)
      cartControllers[i]->sendPosCmd(q_des[i], dq_des[i], dt);  // TODO: update dq

  } else
    RCLCPP_WARN_STREAM(this->get_logger(), "not implemented");

  for (size_t i = 0; i < nRobot; i++)
    feedback(desPose[i], desVel[i], interfaces[cartControllers[i]->getIndex()].interfaces);
}

void OhrcController::updateDesired() {
  for (size_t i = 0; i < nRobot; i++) {
    cartControllers[i]->updateCurState();

    // tf2::transformEigenToKDL(cartControllers[i]->getT_init(), desPose[i]);
    tf2::transformEigenToKDL(cartControllers[i]->getT_cur(), desPose[i]);
    desVel[i] = KDL::Twist();

    updateTargetPose(desPose[i], desVel[i], interfaces[cartControllers[i]->getIndex()]);

    // applyBaseControl(desPose[i], desVel[i], cartControllers[i]);

    cartControllers[i]->setDesired(desPose[i], desVel[i]);
  }

  // preInterfaceProcess(interfaces);
}

bool OhrcController::initRobotPose() {
  if (!std::all_of(cartControllers.begin(), cartControllers.end(), [](auto& c) { return c->isInitialized(); })) {
    updateAllCurState();

    if (unique_state) {
      std::vector<VectorXd> q_des_t_(nRobot), dq_des_t_(nRobot);
      std::vector<KDL::JntArray> q_cur_(nRobot);
      std::vector<double> T_(nRobot), s_(nRobot);
      std::vector<int> lastLoop_(nRobot);

      for (size_t i = 0; i < nRobot; i++)
        cartControllers[i]->getInitCmd(q_des_t_[i], dq_des_t_[i], q_cur_[i], lastLoop_[i], T_[i], s_[i]);

      VectorXd q_des_t_all = math_utility::concatenateVecOfVec(q_des_t_);
      VectorXd dq_des_t_all = math_utility::concatenateVecOfVec(dq_des_t_);
      KDL::JntArray q_cur_all;
      q_cur_all.data = math_utility::concatenateVecOfVec(q_cur_);
      bool lastLoop = std::all_of(lastLoop_.begin(), lastLoop_.end(), [](int x) { return x == 1; });
      double T = *std::max_element(T_.begin(), T_.end());
      double s = *std::max_element(s_.begin(), s_.end());

      cartControllers[0]->sendIntJntCmd(q_des_t_all, dq_des_t_all, q_cur_all, lastLoop, T, s);

    } else {
      for (auto cartController : cartControllers)
        if (!cartController->isInitialized())
          cartController->sendIntJntCmd();
    }

    return false;
  }

  return true;
}

void OhrcController::controlLoop() {
  if (!isStarted) {
    this->starting();
    isStarted = true;
  }

  if (!this->initRobotPose())
    return;  // blocking until all robot is initialized.

  if (!isControllerInitialized) {
    this->initController();
    isControllerInitialized = true;
  }

  rclcpp::Time t = this->get_clock()->now();
  rclcpp::Duration dur = rclcpp::Duration::from_seconds(dt);
  // begin = std::chrono::high_resolution_clock::now();

  this->updateDesired();

  if (IKmode == IKMode::Order) {
    for (size_t i = 0; i < nRobot; i++)
      cartControllers[i]->update(t, dur);
  } else if (IKmode == IKMode::Parallel) {  // parallel IK(multithreading)
    std::vector<std::unique_ptr<std::thread>> workers(nRobot);
    sched_param sch;
    sch.sched_priority = 1;
    for (size_t i = 0; i < nRobot; i++) {
      auto c = cartControllers[i];
      workers[i] = std::make_unique<std::thread>([c, t, dur]() { c->update(t, dur); });
      // pthread_setschedparam(workers[i]->native_handle(), SCHED_FIFO, &sch);
    }
    std::for_each(workers.begin(), workers.end(), [](std::unique_ptr<std::thread>& th) { th->join(); });
  } else if (IKmode == IKMode::Concatenated)
    this->update(t, dur);

  // std::chrono::microseconds t = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - begin);
  // ROS_INFO_STREAM("IK time: " << t.count() * 1.0e-3 << "[ms]");
}

void OhrcController::control() {
  this->node = this->shared_from_this();

  std::vector<std::string> robots, hw_configs;
  if (!this->getRosParams(robots, hw_configs)) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Failed to get the initial parameters. Shutting down...");
    rclcpp::shutdown();
  }

  this->initMenbers(robots, hw_configs);

  exec.spin();

  rclcpp::shutdown();
}
