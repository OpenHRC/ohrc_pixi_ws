#include "ohrc_control/position_feedback_controller.hpp"

void PositionFeedbackController::initInterface() {
}

void PositionFeedbackController::updateTargetPose(const rclcpp::Time t, KDL::Frame& pose, KDL::Twist& twist) {
  // std::cout << "????" << std::endl;
  KDL::Frame frame;
  KDL::Twist vel;
  controller->getCartState(frame, vel);

  VectorXd e = MyIK::getCartError(frame, pose);
  VectorXd v_pi = this->pControl(e, twist);

  // if (controller->getOperationEnable()) {  // still needed? => NO! might need to add interpolation or cartecian velocity limit
  //   v = v_pi;
  // }

  tf2::twistEigenToKDL(v_pi, twist);
}

VectorXd PositionFeedbackController::pControl(const VectorXd& e, const KDL::Twist& twist) {
  const double kp = 2.0;

  Matrix<double, 6, 1> v_des;
  tf2::twistKDLToEigen(twist, v_des);

  VectorXd gain(6);

  for (size_t i = 0; i < 6; i++) {
    gain[i] = kp;
  }

  gain.tail(3) = gain.tail(3) * 0.5 / M_PI;
  return v_des + gain.asDiagonal() * kp * e;  // + ki * e_integ;
}
