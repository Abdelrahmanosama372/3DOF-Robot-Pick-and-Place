#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <pluginlib/class_list_macros.hpp>
#include <vector>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace stmbot_controllers
{
class StmbotInterface : public hardware_interface::SystemInterface
{
public:
  explicit StmbotInterface() = default;
  ~StmbotInterface();

  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev_state) override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    LibSerial::SerialPort serial_port;
    std::string port;
    std::vector<double> position_states;
    std::vector<double> position_commands;
    std::vector<double> prev_position_commands;
};
}  // namespace stmbot_controllers
