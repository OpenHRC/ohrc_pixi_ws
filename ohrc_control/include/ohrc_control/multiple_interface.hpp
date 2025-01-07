#ifndef SWITCHING_INTERFACE_HPP
#define SWITCHING_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include "ohrc_control/interface.hpp"

template <class T1, class T2>
class MultiInterface : public ohrc_controller {
protected:
  void defineInterface() override {
    for (size_t i = 0; i < this->getNRobot(); i++)
      this->interfaces[i] = std::make_shared<DualInterface<T1, T2>>(cartControllers[i]);
  }
}

template <class T1, class T2>
class DualInterface : virtual public T1, virtual public T2 {
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv;
  int cur_interface = 0;

protected:
  void switchInterface() {
    cur_interface = (cur_interface + 1) % 2;

    if (cur_interface == 0)
      T1::resetInterface();
    else
      T2::resetInterface();
  }

  void srvSwitchInterface(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) {
    RCLCPP_WARN(rclcpp::get_logger("SwitchingInterface"), "/switch_interface was called");
    switchInterface();
  }

public:
  SwitchingInterface(const std::shared_ptr<CartController> controller) : T1(controller), T2(controller), Interface(controller) {
    srv = T1::get_node()->create_service<std_srvs::srv::Empty>("/switch_interface",
                                                               std::bind(&SwitchingInterface::srvSwitchInterface, this, std::placeholders::_1, std::placeholders::_2));
  }

  void updateTargetPose(KDL::Frame &pose, KDL::Twist &twist) override {
    if (cur_interface == 0)
      T1::updateTargetPose(pose, twist);
    else
      T2::updateTargetPose(pose, twist);
  }

  void initInterface() override {
    T1::initInterface();
    T2::initInterface();
  };

  void resetInterface() override {
    T1::resetInterface();
    T2::resetInterface();
  };
};

#endif  // SWITCHING_INTERFACE_HPP
