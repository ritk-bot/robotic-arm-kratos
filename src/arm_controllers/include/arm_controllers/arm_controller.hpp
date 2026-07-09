#pragma once

#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


namespace arm_controllers
{

class ArmJogController : public controller_interface::ControllerInterface
{
public:

    ArmJogController();

    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::InterfaceConfiguration
    command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration
    state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time &,
        const rclcpp::Duration &) override;


private:

    enum JointIndex
    {
        BASE = 0,
        SHOULDER,
        ELBOW,
        PITCH,
        ROLL,
        GRIPPER,
        NUM_JOINTS
    };

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
        velocity_sub_;

    std::vector<double> commanded_velocities_;

    std::vector<double> target_positions_;

    rclcpp::Time last_command_time_;

    double command_timeout_ = 0.25;

    std::vector<double> lower_limits_;

    std::vector<double> upper_limits_;

    static constexpr double MAX_JOG_SPEED = 1.5;

    void velocity_callback(
        const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    };

} // namespace arm_controllers