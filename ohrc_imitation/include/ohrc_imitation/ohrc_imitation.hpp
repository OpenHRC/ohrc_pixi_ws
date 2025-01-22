#ifndef OHRC_IMITATION_HPP
#define OHRC_IMITATION_HPP

#include "ohrc_control/ohrc_controller.hpp"

class ImitationController : virtual public OhrcController {
protected:
  //   void initControllerAdditional() override {};

  virtual void imitationDesired(KDL::Frame& desPose, KDL::Twist& desVel, const int idx, const KDL::Frame& desPoseRef, const KDL::Twist& desVelRef, const int idxRef) {
    desPose = desPoseRef;
    desVel = desVelRef;
  }

  void overrideDesired(std::vector<KDL::Frame>& desPose, std::vector<KDL::Twist>& desVel) override {
    // find index of true in the vector
    // int idx = std::distance(priorityIdx.begin(), std::find(priorityIdx.begin(), priorityIdx.end(), true));
    int idx = priorityIdx;

    if (idx > -1 && idx < this->getNRobot())
      for (size_t i = 0; i < this->getNRobot(); i++) {
        if (i == idx)
          continue;

        imitationDesired(desPose[i], desVel[i], i, desPose[idx], desVel[idx], idx);
      }
  }
};

#endif  // OHRC_IMITATION_HPP
