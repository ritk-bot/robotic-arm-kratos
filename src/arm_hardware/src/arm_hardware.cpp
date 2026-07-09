#include "arm_hardware/arm_hardware.hpp"

#include <cstring>

#include "pluginlib/class_list_macros.hpp"

namespace arm_hardware
{

hardware_interface::CallbackReturn
ArmHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if(SystemInterface::on_init(info)
        != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_positions_.assign(NUM_JOINTS,0.0);
    hw_commands_.assign(NUM_JOINTS,0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ArmHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;

    for(size_t i=0;i<NUM_JOINTS;i++)
    {
        interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_positions_[i]);
    }

    return interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArmHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;

    for(size_t i=0;i<NUM_JOINTS;i++)
    {
        interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_commands_[i]);
    }

    return interfaces;
}

bool ArmHardware::configure_serial_port()
{
    termios tty{};

    if(tcgetattr(serial_fd_, &tty) != 0)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("ArmHardware"),
            "tcgetattr() failed.");

        return false;
    }

    cfsetospeed(&tty, baudrate_);
    cfsetispeed(&tty, baudrate_);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_lflag = 0;
    tty.c_iflag = 0;
    tty.c_oflag = 0;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if(tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("ArmHardware"),
            "tcsetattr() failed.");

        return false;
    }

    return true;
}

hardware_interface::CallbackReturn
ArmHardware::on_activate(
    const rclcpp_lifecycle::State &)
{
    hw_positions_ = hw_commands_;

    serial_fd_ = open(
        serial_port_.c_str(),
        O_RDWR | O_NOCTTY | O_SYNC);

    if(serial_fd_ < 0)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("ArmHardware"),
            "Could not open %s",
            serial_port_.c_str());

        return hardware_interface::CallbackReturn::ERROR;
    }

    if(!configure_serial_port())
    {
        close(serial_fd_);
        serial_fd_ = -1;

        return hardware_interface::CallbackReturn::ERROR;
    }
    tcflush(serial_fd_, TCIOFLUSH);
    RCLCPP_INFO(
        rclcpp::get_logger("ArmHardware"),
        "Opened UART %s",
        serial_port_.c_str());

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ArmHardware::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    if(serial_fd_ >= 0)
    {
        close(serial_fd_);
        serial_fd_ = -1;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("ArmHardware"),
        "Hardware Deactivated");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
ArmHardware::read(
    const rclcpp::Time &,
    const rclcpp::Duration &)
{
    // Fake perfect hardware for now

    hw_positions_ = hw_commands_;

    return hardware_interface::return_type::OK;
}

bool ArmHardware::send_packet(
    const ArmCommandPacket & packet)
{
    const uint8_t * ptr =
        reinterpret_cast<const uint8_t*>(&packet);

    size_t remaining = sizeof(packet);

    while(remaining > 0)
    {
        ssize_t written =
            ::write(serial_fd_, ptr, remaining);

        if(written < 0)
        {
            return false;
        }

        ptr += written;
        remaining -= written;
    }

    return true;
}

hardware_interface::return_type
ArmHardware::write(
    const rclcpp::Time &,
    const rclcpp::Duration &)
{
    ArmCommandPacket packet;

    for(size_t i=0;i<NUM_JOINTS;i++)
    {
        packet.target_position[i] =
            hw_commands_[i];
    }

    if(!send_packet(packet))
    {
    RCLCPP_WARN(
        rclcpp::get_logger("ArmHardware"),
       "UART transmission failed.");
    }

    return hardware_interface::return_type::OK;
}

} // namespace arm_hardware

PLUGINLIB_EXPORT_CLASS(
    arm_hardware::ArmHardware,
    hardware_interface::SystemInterface
)