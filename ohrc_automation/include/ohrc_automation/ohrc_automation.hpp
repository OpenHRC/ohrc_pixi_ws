#ifndef OHRC_AUTOMATION_HPP
#define OHRC_AUTOMATION_HPP

#include "ohrc_control/ohrc_controller.hpp"

class OhrcAutomation {
  ros::Subscriber subTarget;

  enum struct TaskState {
    OnGoing,
    Success,
    Fail,
  };
}

#endif  // OHRC_AUTOMATION_HPP