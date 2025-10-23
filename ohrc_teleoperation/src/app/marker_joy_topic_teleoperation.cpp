
#include "ohrc_control/ohrc_controller.hpp"
#include "ohrc_teleoperation/joy_topic_interface.hpp"
#include "ohrc_teleoperation/marker_interface.hpp"
#include "ohrc_teleoperation/state_topic_interface.hpp"

class MultiInterface : public OhrcController {
protected:
  void defineInterface() override {
    for (size_t i = 0; i < this->getNRobot(); i++) {
      this->interfaces[i].interfaces.push_back(std::make_shared<MarkerInterface>(cartControllers[i]));
      this->interfaces[i].interfaces.push_back(std::make_shared<JoyTopicInterface>(cartControllers[i]));
      this->interfaces[i].interfaces.push_back(std::make_shared<StateTopicInterface>(cartControllers[i]));
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto interface = std::make_shared<MultiInterface>();
  interface->control();

  return 0;
}