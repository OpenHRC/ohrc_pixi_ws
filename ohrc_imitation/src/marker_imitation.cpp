#include "ohrc_imitation/single_imitation_controller.hpp"
#include "ohrc_teleoperation/marker_interface.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto interface = std::make_shared<SingleImitationController<MarkerInterface>>();
  interface->control();

  return 0;
}