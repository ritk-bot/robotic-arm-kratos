#pragma once

#include <vector>
#include <string>
#include <cstdint>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"

#include "rclcpp/rclcpp.hpp"

namespace arm_hardware
{

//====================================================
// Packet Definitions
//====================================================

enum class PacketType : uint8_t
{
    ARM_COMMAND = 0x01,
    ARM_STATE   = 0x02,
    HEARTBEAT   = 0x03
};

#pragma pack(push,1)

struct ArmCommandPacket
{
    uint8_t sync1 = 0xAA;
    uint8_t sync2 = 0x55;

    uint8_t packet_type =
        static_cast<uint8_t>(PacketType::ARM_COMMAND);

    double target_position[6];

    uint16_t crc = 0;
};

#pragma pack(pop)

//====================================================
// Hardware Interface
//====================================================

class ArmHardware :
    public hardware_interface::SystemInterface
{
public:

    ArmHardware() = default;

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &) override;

    hardware_interface::return_type read(
        const rclcpp::Time &,
        const rclcpp::Duration &) override;

    hardware_interface::return_type write(
        const rclcpp::Time &,
        const rclcpp::Duration &) override;

private:

    //------------------------------------------------
    // Constants
    //------------------------------------------------

    static constexpr size_t NUM_JOINTS = 6;

    //------------------------------------------------
    // Joint Data
    //------------------------------------------------

    std::vector<double> hw_positions_;

    std::vector<double> hw_commands_;

    //------------------------------------------------
    // UART
    //------------------------------------------------

    int serial_fd_ = -1;

    std::string serial_port_ = "/dev/ttyUSB0";

    speed_t baudrate_ = B115200;

    //------------------------------------------------
    // UART Helpers
    //------------------------------------------------

    bool configure_serial_port();

    bool send_packet(
        const ArmCommandPacket & packet);

};

} // namespace arm_hardware