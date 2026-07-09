from launch import LaunchDescription

from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("arm"),
            "urdf",
            "my_custom_arm_jog.urdf.xacro"
        ])
    ])


    controller_config = PathJoinSubstitution([
        FindPackageShare("arm"),
        "config",
        "arm_controller.yaml"
    ])


    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description
            }
        ]
    )


    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description
            },
            controller_config
        ]
    )


    joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            "joint_state_broadcaster"
        ],
        output="screen"
    )


    arm_jog_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            "arm_jog_controller"
        ],
        output="screen"
    )


    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster,
        arm_jog_controller,
    ])