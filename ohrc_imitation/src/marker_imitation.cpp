
#include "ohrc_control/single_interface.hpp"
#include "ohrc_imitation/ohrc_imitation.hpp"
#include "ohrc_teleoperation/marker_interface.hpp"

template <class T>
class SingleImitationController : virtual public SingleInterface<T>, virtual public ImitationController {};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto interface = std::make_shared<SingleImitationController<MarkerInterface>>();
  interface->control();

  return 0;
}