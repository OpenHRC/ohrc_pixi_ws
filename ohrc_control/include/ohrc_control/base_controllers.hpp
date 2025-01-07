#ifndef BASE_CONTROLLERS_HPP
#define BASE_CONTROLLERS_HPP

#include "ohrc_control/admittance_controller.hpp"
#include "ohrc_control/hybrid_feedback_controller.hpp"
#include "ohrc_control/no_feedback_controller.hpp"
#include "ohrc_control/ohrc_control.hpp"
#include "ohrc_control/position_feedback_controller.hpp"

namespace ohrc_control {
std::shared_ptr<Interface> selectBaseController(FeedbackMode feedbackMode, std::shared_ptr<CartController> cartController) {
  std::shared_ptr<Interface> baseController;
  if (cartController->getFtFound() && feedbackMode == FeedbackMode::Admittance)
    baseController = std::make_shared<AdmittanceController>(cartController);
  else if (feedbackMode == FeedbackMode::PositionFeedback)
    baseController = std::make_shared<PositionFeedbackController>(cartController);
  else if (feedbackMode == FeedbackMode::HybridFeedback)
    baseController = std::make_shared<HybridFeedbackController>(cartController);
  else if (feedbackMode == FeedbackMode::NoFeedback)
    baseController = std::make_shared<NoFeedbackController>(cartController);
  else {
    RCLCPP_WARN_STREAM(cartController->get_logger(), "No feedback controller selected. Using NoFeedbackController");
    baseController = std::make_shared<NoFeedbackController>(cartController);
  }
  return baseController;
}
};  // namespace ohrc_control

#endif  // BASE_CONTROLLERS_HPP