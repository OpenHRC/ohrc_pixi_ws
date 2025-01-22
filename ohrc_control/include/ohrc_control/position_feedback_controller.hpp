#ifndef POSITION_FEEDBACK_CONTROLLER_HPP
#define POSITION_FEEDBACK_CONTROLLER_HPP

#include "ohrc_control/interface.hpp"

class PositionFeedbackController : public Interface {
  VectorXd pControl(const VectorXd& e, const KDL::Twist& twist);

public:
  using Interface::Interface;

  virtual void updateTargetPose(const rclcpp::Time t, KDL::Frame& pose, KDL::Twist& twist) override;
  virtual void initInterface() override;
};

#endif  // POSITION_FEEDBACK_CONTROLLER_HPP
