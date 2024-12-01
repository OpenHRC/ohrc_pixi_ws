#ifndef SINGLE_INTERFACE_H
#define SINGLE_INTERFACE_H

#include "ohrc_control/ohrc_controller.hpp"

template <class T>
class SingleInterface : public OhrcController {
protected:
  void defineInterface() override {
    for (int i = 0; i < this->getNRobot(); i++)
      this->interfaces[i] = std::make_shared<T>(cartControllers[i]);
  }
  // public:
  //   SingleInterface() {
  //     for (int i = 0; i < this->getNRobot(); i++)
  //       this->interfaces[i] = std::make_shared<T>(cartControllers[i]);
  //   }
};

#endif  // SINGLE_INTERFACE_H