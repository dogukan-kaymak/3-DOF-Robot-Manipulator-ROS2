#include "mechanism_308_hardware/mechanism_308_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mechanism_308_hardware {

Mechanism308SystemHardware::~Mechanism308SystemHardware() {
  executor_.cancel();
  if (spin_thread_ && spin_thread_->joinable()) {
    spin_thread_->join();
  }
}

hardware_interface::CallbackReturn Mechanism308SystemHardware::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Setup internal variables
  hw_positions_.resize(info_.joints.size(),
                       std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(),
                        std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(),
                      std::numeric_limits<double>::quiet_NaN());
  last_published_commands_.resize(info_.joints.size(),
                                  std::numeric_limits<double>::quiet_NaN());

  // Initialize ROS Node
  node_ = std::make_shared<rclcpp::Node>("mechanism_308_hardware_node");

  // Create publishers and subscribers for each joint
  for (size_t i = 0; i < info_.joints.size(); i++) {
    // Our joints are r1, r2, r3 -> Map to /motor1/, /motor2/, /motor3/
    std::string topic_prefix = "/motor" + std::to_string(i + 1);

    // ESP32 publishes Best Effort, so subscriber must be Best Effort
    rclcpp::QoS sub_qos(10);
    sub_qos.best_effort();
    auto sub = node_->create_subscription<std_msgs::msg::Float32>(
        topic_prefix + "/position", sub_qos,
        [this, i](const std_msgs::msg::Float32::SharedPtr msg) {
          this->position_callback(msg, i);
        });
    pos_subs_.push_back(sub);

    // Publisher for target — BEST_EFFORT: ACK gerektirmez, Wi-Fi'yi yormaz
    rclcpp::QoS pub_qos(10);
    pub_qos.best_effort();
    auto pub = node_->create_publisher<std_msgs::msg::Float32>(
        topic_prefix + "/target", pub_qos);
    target_pubs_.push_back(pub);
  }

  // Start executor thread to process callbacks
  executor_.add_node(node_);
  spin_thread_ = std::make_shared<std::thread>([this]() { executor_.spin(); });

  return hardware_interface::CallbackReturn::SUCCESS;
}

void Mechanism308SystemHardware::position_callback(
    const std_msgs::msg::Float32::SharedPtr msg, int motor_idx) {
  // Motorlar fiziksel olarak ters dondugu icin pozisyonu eksi (-) ile carparak MoveIt'e veriyoruz
  hw_positions_[motor_idx] = -msg->data;
}

std::vector<hardware_interface::StateInterface>
Mechanism308SystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Mechanism308SystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn Mechanism308SystemHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  for (auto &pos : hw_positions_) {
    if (std::isnan(pos))
      pos = 0.0;
  }
  for (auto &vel : hw_velocities_) {
    vel = 0.0;
  }
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    hw_commands_[i] = hw_positions_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("Mechanism308SystemHardware"),
              "Real Hardware Interface activated! Listening to ESP32...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Mechanism308SystemHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("Mechanism308SystemHardware"),
              "Hardware Interface deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
Mechanism308SystemHardware::read(const rclcpp::Time & /*time*/,
                                 const rclcpp::Duration & /*period*/) {
  // Position is updated in the callback asynchronously.
  // MoveIt will read from hw_positions_ directly via pointers.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
Mechanism308SystemHardware::write(const rclcpp::Time & /*time*/,
                                  const rclcpp::Duration & /*period*/) {
  // THROTTLE: Only publish every 33rd cycle (3 Hz) to prevent ESP32 UART log
  // overflow! MoveIt sends trajectories at 100Hz.
  static int throttle_counter = 0;
  throttle_counter++;
  if (throttle_counter >= 33) {
    throttle_counter = 0;
    // Write command to ESP32
    for (size_t i = 0; i < hw_commands_.size(); i++) {
      if (std::isnan(hw_commands_[i]))
        continue;

      std_msgs::msg::Float32 msg;
      // Motorlar fiziksel olarak ters baglandigi/monte edildigi icin hedefi ters ceviriyoruz
      msg.data = -hw_commands_[i];
      target_pubs_[i]->publish(msg);
      last_published_commands_[i] = hw_commands_[i];
    }
  }
  return hardware_interface::return_type::OK;
}

} // namespace mechanism_308_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mechanism_308_hardware::Mechanism308SystemHardware,
                       hardware_interface::SystemInterface)
