#include "ohrc_teleoperation/marker_interface.hpp"

void MarkerInterface::initInterface() {
  interfaceName = "MarkerInterface";
  RclcppUtility::declare_and_get_parameter_enum(this->node, interfaceName + ".feedback_mode", FeedbackMode::PositionFeedback, feedbackMode);

  server = std::make_unique<interactive_markers::InteractiveMarkerServer>(robot_ns + "eef_marker", node, rclcpp::QoS(1));
  configMarker();

  controller->updatePosFilterCutoff(10.0);

  _markerPose = tf2::toMsg(controller->getT_cur());

  th = std::thread(&MarkerInterface::markerThread, this);

  controller->enableOperation();
}

void MarkerInterface::configMarker() {
  // set initial marker config
  int_marker.header.frame_id = controller->getChainStart();
  int_marker.header.stamp = rclcpp::Time(0);
  int_marker.pose = tf2::toMsg(controller->getT_cur());
  int_marker.scale = 0.1;
  int_marker.name = robot_ns;

  // insert a box
  visualization_msgs::msg::Marker box_marker;
  box_marker.type = visualization_msgs::msg::Marker::CUBE;
  box_marker.scale.x = int_marker.scale * 0.45;
  box_marker.scale.y = int_marker.scale * 0.45;
  box_marker.scale.z = int_marker.scale * 0.45;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  visualization_msgs::msg::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back(box_marker);

  int_marker.controls.push_back(box_control);
  int_marker.controls[0].interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;

  visualization_msgs::msg::InteractiveMarkerControl control;

  const std::string axis_name[3] = { "x", "y", "z" };
  const Eigen::Quaterniond quat[3] = { Eigen::Quaterniond(1, 1, 0, 0), Eigen::Quaterniond(1, 0, 0, 1), Eigen::Quaterniond(1, 0, 1, 0) };

  for (size_t i = 0; i < 3; i++) {
    control.orientation = tf2::toMsg(quat[i].normalized());
    control.name = "rotate_" + axis_name[i];
    control.always_visible = true;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_" + axis_name[i];
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  // add the interactive marker
  server->insert(int_marker, std::bind(&MarkerInterface::processFeedback, this, std::placeholders::_1, node->get_logger()));

  // 'commit' changes and send to all clients
  server->applyChanges();

  RCLCPP_INFO_STREAM(node->get_logger(), "Set interactive marker: " << robot_ns << "eef_marker");
}

void MarkerInterface::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback, rclcpp::Logger logger) {
  std::lock_guard<std::mutex> lock(mtx);
  subFirst = true;
  _feedback = *feedback;
  // t_prev = this->get_clock()->now();
}

// void MarkerInterface::updateInterface() {
void MarkerInterface::markerThread() {
  rclcpp::Rate rate(30);
  while (interfaceRunning && rclcpp::ok()) {
    {
      std::lock_guard<std::mutex> lock(mtx);
      if (subFirst) {
        // _markerDt = (this->get_clock()->now() - t_prev).toSec();
        if (updateIsEnable(_feedback.event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP)) {
          _markerPose = _feedback.pose;
          count_disable = 1;
          _flagSubInteractiveMarker = true;
        } else {
          _flagSubInteractiveMarker = false;
          server->setPose(int_marker.name, tf2::toMsg(controller->getT_cur()));
          server->applyChanges();
          // subFirst = true;
        }
      }
    }

    rate.sleep();
  }
}

void MarkerInterface::updateTargetPose(const rclcpp::Time t, KDL::Frame& pose, KDL::Twist& twist) {
  geometry_msgs::msg::Pose markerPose;
  double markerDt;
  {
    std::lock_guard<std::mutex> lock(mtx);
    if (!_flagSubInteractiveMarker) {
      return;
    }

    markerPose = _markerPose;

    // _flagSubInteractiveMarker = false;
  }

  tf2::fromMsg(markerPose, pose);
}

void MarkerInterface::resetInterface() {
  server->setPose(int_marker.name, tf2::toMsg(controller->getT_cur()));
  server->applyChanges();
  subFirst = false;
}
