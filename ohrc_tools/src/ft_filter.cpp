#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ohrc_msgs/msg/contact.hpp>

#include <Eigen/Geometry>
#include <mutex>

#include "ohrc_common/geometry_msgs_utility/geometry_msgs_utility.h"

#include "ohrc_common/rclcpp_utility.hpp"

using namespace std::placeholders;

class FTFilter : public rclcpp::Node
{
  std::shared_ptr<rclcpp::Node> node;

  std::unique_ptr<geometry_msgs_utility::WrenchStamped> forceLpf, forceLpf_;

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub;

  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service;
  rclcpp::CallbackGroup::SharedPtr callback_group;

  geometry_msgs_utility::Wrench::paramLPF paramLpf;
  geometry_msgs_utility::paramDeadZone paramDeadZone;

  unsigned int count = 1;

  Eigen::VectorXd offset = Eigen::VectorXd::Zero(6);

  std::mutex mtx;
  geometry_msgs::msg::WrenchStamped raw_;

  bool reseted = false;
  bool flag_ft = false;

  void cbForce(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx);
    raw_ = *msg;
    flag_ft = true;
  }

  bool reset_offset(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    {
      std::lock_guard<std::mutex> lock(mtx);
      reseted = false;
      count = 1;
      offset = Eigen::VectorXd::Zero(6);
      RCLCPP_INFO(this->get_logger(), "Resetting ft sensor offset...");
    }

    while (rclcpp::ok())
    {
      {
        std::lock_guard<std::mutex> lock(mtx);
        if (reseted)
          break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO(this->get_logger(), "Reseted ft sensor offset.");

    res->success = true;
    res->message = "Reseted ft sensor offset.";
    return true;
  }

  void filter_cb()
  {
    // std::cout << offset.transpose() << std::endl;
    geometry_msgs::msg::WrenchStamped raw, raw_dz, filtered;
    {
      std::lock_guard<std::mutex> lock(mtx);
      if (!flag_ft)
        return;
      raw = raw_;
    }

    if (count < paramLpf.sampling_freq * 3.0 && !reseted)
    {  // 3 seconds
      offset = (offset * (count - 1) + tf2::fromMsg(raw.wrench)) / count;
      count++;
      return;
    }
    else if (!reseted)
    {
      reseted = true;
      RCLCPP_INFO_STREAM(this->get_logger(), "Offset was reset.");
    }

    raw_dz.header = raw.header;

    tf2::toMsg(tf2::fromMsg(raw.wrench) - offset, raw_dz.wrench);
    forceLpf_->LPF(raw_dz, raw_dz);
    forceLpf->deadZone_LPF(raw_dz, filtered);

    pub->publish(filtered);
  }

public:
  FTFilter() : Node("ft_sensor_filter")
  {
    node = std::shared_ptr<rclcpp::Node>(this);
    callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    std::string topic_name_raw;
    if (!RclcppUtility::declare_and_get_parameter<std::string>(node, "ft_in", "ft_sensor/raw", topic_name_raw))
      RCLCPP_ERROR(this->get_logger(), "Failed to get ft sensor rate");

    std::string topic_name_filtered;
    if (!RclcppUtility::declare_and_get_parameter<std::string>(node, "ft_out", "ft_sensor/filtered",
                                                               topic_name_filtered))
      RCLCPP_ERROR(this->get_logger(), "Failed to get ft sensor rate");

    if (!RclcppUtility::declare_and_get_parameter<double>(node, "sampling_freq", 1000.0, paramLpf.sampling_freq))
      RCLCPP_ERROR(this->get_logger(), "Failed to get ft sensor rate");

    if (!RclcppUtility::declare_and_get_parameter<double>(node, "cutoff_freq", 300.0, paramLpf.cutoff_freq))
      RCLCPP_ERROR(this->get_logger(), "Failed to get cutoff_freq setting");

    if (!RclcppUtility::declare_and_get_parameter<int>(node, "lpf_order", 2, paramLpf.order))
      RCLCPP_ERROR(this->get_logger(), "Failed to get lpf_order setting");

    if (!RclcppUtility::declare_and_get_parameter<double>(node, "deadzone.force.lower", -0.1,
                                                          paramDeadZone.force_lower))
      RCLCPP_ERROR(this->get_logger(), "Failed to get deadzone/force/lower setting");

    if (!RclcppUtility::declare_and_get_parameter<double>(node, "deadzone.force.upper", 0.1, paramDeadZone.force_upper))
      RCLCPP_ERROR(this->get_logger(), "Failed to get deadzone/force/upper setting");

    if (!RclcppUtility::declare_and_get_parameter<double>(node, "deadzone.torque.lower", -0.1,
                                                          paramDeadZone.torque_lower))
      RCLCPP_ERROR(this->get_logger(), "Failed to get deadzone/torque/lower setting");

    if (!RclcppUtility::declare_and_get_parameter<double>(node, "deadzone.torque.upper", 0.1,
                                                          paramDeadZone.torque_upper))
      RCLCPP_ERROR(this->get_logger(), "Failed to get deadzone/torque/upper setting");

    forceLpf = std::make_unique<geometry_msgs_utility::WrenchStamped>(paramLpf, paramDeadZone);
    forceLpf_.reset(new geometry_msgs_utility::WrenchStamped(paramLpf, paramDeadZone));

    service =
        node->create_service<std_srvs::srv::Trigger>("reset_offset", std::bind(&FTFilter::reset_offset, this, _1, _2),
                                                     rmw_qos_profile_services_default, callback_group);

    sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(topic_name_raw, rclcpp::QoS(1),
                                                                       std::bind(&FTFilter::cbForce, this, _1));
    pub = node->create_publisher<geometry_msgs::msg::WrenchStamped>(topic_name_filtered, 1);

    timer = this->create_wall_timer(
        rclcpp::Duration::from_seconds(1.0 / paramLpf.sampling_freq).to_chrono<std::chrono::nanoseconds>(),
        std::bind(&FTFilter::filter_cb, this));
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto filter = std::make_shared<FTFilter>();
  exec.add_node(filter);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
