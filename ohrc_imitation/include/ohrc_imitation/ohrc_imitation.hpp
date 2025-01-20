#ifndef OHRC_IMITATION_HPP
#define OHRC_IMITATION_HPP

#include "ohrc_control/ohrc_controller.hpp"

class ImitationController : virtual public OhrcController {
protected:
  void initControllerAdditional() override {

  };

  void overrideDesired(std::vector<KDL::Frame>& desPose, std::vector<KDL::Twist>& desVel) override {
    // find index of true in the vector
    int idx = std::distance(priorityIdx.begin(), std::find(priorityIdx.begin(), priorityIdx.end(), true));

    for (size_t i = 0; i < this->getNRobot(); i++) {
      if (i == idx)
        continue;

      desPose[i] = desPose[idx];
      desVel[i] = desVel[idx];
    };
  }
};

#endif  // OHRC_IMITATION_HPP