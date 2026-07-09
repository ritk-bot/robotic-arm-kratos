#include "arm_controllers/arm_controller.hpp"

#include <algorithm>

#include "pluginlib/class_list_macros.hpp"

namespace arm_controllers
{

ArmJogController::ArmJogController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn

ArmJogController::on_activate(
    const rclcpp_lifecycle::State &)
{
    for(size_t i=0;i<NUM_JOINTS;i++)
    {
        target_positions_[i] =
            state_interfaces_[i].get_value();
    }

    RCLCPP_INFO(
        get_node()->get_logger(),
        "Arm Jog Controller Activated");

    return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn

ArmJogController::on_init()
{
    lower_limits_ ={
        -3.14,
        -1.57,
        -2.50,
        -1.57,
        -3.14,
        -1.00
    };

    upper_limits_ = {
        3.14,
        1.57,
        2.50,
        1.57,
        3.14,
        1.00
    };
    last_command_time_ = get_node()->now();
    commanded_velocities_.assign(NUM_JOINTS, 0.0);
    target_positions_.assign(NUM_JOINTS, 0.0);

    velocity_sub_ =
        get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/arm_cmd_vel",
            10,
            std::bind(
                &ArmJogController::velocity_callback,
                this,
                std::placeholders::_1));

    RCLCPP_INFO(
        get_node()->get_logger(),
        "Arm Jog Controller initialized.");

    return controller_interface::CallbackReturn::SUCCESS;
}

void ArmJogController::velocity_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if(msg->data.size() != NUM_JOINTS)
    {
        RCLCPP_WARN(
            get_node()->get_logger(),
            "Expected %d joint velocities, got %ld",
            NUM_JOINTS,
            msg->data.size());

        return;
    }
    last_command_time_ = get_node()->now();
    for(size_t i=0;i<NUM_JOINTS;++i)
    {
        commanded_velocities_[i] = std::clamp(msg->data[i],-MAX_JOG_SPEED,MAX_JOG_SPEED);
    }
}

controller_interface::InterfaceConfiguration

ArmJogController::command_interface_configuration() const
{
    return
    {
        controller_interface::interface_configuration_type::INDIVIDUAL,

        {
            "base_yaw_joint/position",
            "shoulder_joint/position",
            "elbow_joint/position",
            "wrist_pitch_joint/position",
            "wrist_roll_joint/position",
            "gripper_servo_joint/position"
        }
    };
}

controller_interface::InterfaceConfiguration

ArmJogController::state_interface_configuration() const
{
    return
    {
        controller_interface::interface_configuration_type::INDIVIDUAL,

        {
            "base_yaw_joint/position",
            "shoulder_joint/position",
            "elbow_joint/position",
            "wrist_pitch_joint/position",
            "wrist_roll_joint/position",
            "gripper_servo_joint/position"
        }
    };
}

controller_interface::return_type

ArmJogController::update(
    const rclcpp::Time &,
    const rclcpp::Duration & period)
{
    const double dt = period.seconds();

    if ((get_node()->now() - last_command_time_).seconds() > command_timeout_){
        std::fill(commanded_velocities_.begin(), commanded_velocities_.end(), 0.0);
    }
    
    for(size_t i=0;i<NUM_JOINTS;++i)
    {
        target_positions_[i] +=
            commanded_velocities_[i] * dt;
    }


    for(size_t i=0;i<NUM_JOINTS;++i)
    {
    target_positions_[i] =
        std::clamp(
            target_positions_[i],
            lower_limits_[i],
            upper_limits_[i]);
    }

    // Differential wrist
    const double left_bevel =
        target_positions_[PITCH] +
        target_positions_[ROLL];

    const double right_bevel =
        target_positions_[PITCH] -
        target_positions_[ROLL];

    (void)command_interfaces_[BASE]
        .set_value(target_positions_[BASE]);

    (void)command_interfaces_[SHOULDER]
        .set_value(target_positions_[SHOULDER]);

    (void)command_interfaces_[ELBOW]
        .set_value(target_positions_[ELBOW]);

    (void)command_interfaces_[PITCH]
        .set_value(left_bevel);

    (void)command_interfaces_[ROLL]
        .set_value(right_bevel);

    (void)command_interfaces_[GRIPPER]
        .set_value(target_positions_[GRIPPER]);

    return controller_interface::return_type::OK;
}

} // namespace arm_controllers

PLUGINLIB_EXPORT_CLASS(
    arm_controllers::ArmJogController,
    controller_interface::ControllerInterface
)