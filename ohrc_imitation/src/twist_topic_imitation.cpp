#include "ohrc_imitation/single_imitation_controller.hpp"
#include "ohrc_teleoperation/twist_topic_interface.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto interface = std::make_shared<SingleImitationController<TwistTopicInterface>>();
  interface->control();

  return 0;
}