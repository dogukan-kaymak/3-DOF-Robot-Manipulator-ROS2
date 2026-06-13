#pragma once

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace mechanism_308_hardware
{
class Mechanism308SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Mechanism308SystemHardware)

  ~Mechanism308SystemHardware();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ROS Node for Pub/Sub
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<std::thread> spin_thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // Publishers and Subscribers
  std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> target_pubs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> pos_subs_;

  // Hardware state
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> last_published_commands_;

  void position_callback(const std_msgs::msg::Float32::SharedPtr msg, int motor_idx);
};
}  // namespace mechanism_308_hardware
