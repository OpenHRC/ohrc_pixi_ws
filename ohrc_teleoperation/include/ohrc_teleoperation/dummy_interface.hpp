#ifndef DUMMY_INTERFACE_HPP
#define DUMMY_INTERFACE_HPP

#include "ohrc_control/interface.hpp"

class DummyInterface : public Interface {
public:
  using Interface::Interface;

  virtual void initInterface() override {
    std::string interfaceName = "DummyInterface";
    RclcppUtility::declare_and_get_parameter_enum(this->node, interfaceName + ".feedback_mode", FeedbackMode::NoFeedback, feedbackMode);
  };
};

#endif  // DUMMY_INTERFACE_HPP