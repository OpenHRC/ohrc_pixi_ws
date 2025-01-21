#ifndef SINGLE_IMITATION_CONTROLLER_HPP
#define SINGLE_IMITATION_CONTROLLER_HPP

#include "ohrc_control/single_interface.hpp"
#include "ohrc_imitation/ohrc_imitation.hpp"

template <class T>
class SingleImitationController : virtual public SingleInterface<T>, virtual public ImitationController {};

#endif  // SINGLE_IMITATION_CONTROLLER_HPP