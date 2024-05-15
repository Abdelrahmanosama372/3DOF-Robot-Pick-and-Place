#include "stmbot_interface.hpp"


namespace stmbot_controllers
{

StmbotInterface::~StmbotInterface()
{
    if(serial_port.IsOpen()){
        try
        {
            serial_port.Close();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("StmbotInterface"),"Error closing the serial port " << port);
        }
        
    }
}

CallbackReturn StmbotInterface::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if(result != CallbackReturn::SUCCESS){
        return result;
    }

    try
    {
        port = info_.hardware_parameters.at("port");
    }
    catch(const std::exception& e)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("StmbotInterface"),"no serial port provided aborted");
        return CallbackReturn::FAILURE;
    }

    position_states.reserve(info_.joints.size());
    position_commands.reserve(info_.joints.size());
    prev_position_commands.reserve(info_.joints.size());

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> StmbotInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for(size_t i=0; i<info_.joints.size(); i++){
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &position_states[i]));
    } 
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> StmbotInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for(size_t i=0; i<info_.joints.size(); i++){
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &position_commands[i]));
    } 
    return command_interfaces;
}

CallbackReturn StmbotInterface::on_activate(const rclcpp_lifecycle::State& prev_state)
{
    RCLCPP_INFO(rclcpp::get_logger("StmbotInterface"), "starting the robot hardware");
    try
    {
        serial_port.Open(port);
        serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch(...)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("StmbotInterface"),"Error opening the serial port " << port);
    }

    RCLCPP_INFO(rclcpp::get_logger("StmbotInterface"),"Hardware started, ready to take commands");

    position_states = {0.0,0.0,0.0,0.0};
    prev_position_commands = {0.0,0.0,0.0,0.0};
    position_commands = {0.0,0.0,0.0,0.0};

    return CallbackReturn::SUCCESS;
    
}

CallbackReturn StmbotInterface::on_deactivate(const rclcpp_lifecycle::State& prev_state)
{
    RCLCPP_INFO(rclcpp::get_logger("StmbotInterface"), "deactivating the robot hardware");
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type StmbotInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    position_states = position_commands;
}

hardware_interface::return_type StmbotInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    // if(prev_position_commands == position_commands){
    //     return hardware_interface::return_type::OK;
    // }

    std::string msg = "h";
    try
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("StmbotInterface"), "Sending new command " << msg);
        serial_port.Write(msg);
    }
    catch (...)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("StmbotInterface"), "Error sending the message " << msg << " to the port " << port);
        return hardware_interface::return_type::ERROR;
    }
    prev_position_commands = position_commands;
    return hardware_interface::return_type::OK;
}

}  // namespace stmbot_controllers

PLUGINLIB_EXPORT_CLASS(stmbot_controllers::StmbotInterface,hardware_interface::SystemInterface);