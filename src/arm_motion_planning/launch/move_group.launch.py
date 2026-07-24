from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from ament_index_python.packages import get_package_share_directory
import os
arm_path = get_package_share_directory("arm")

def generate_launch_description():
    moveit_config = (
    MoveItConfigsBuilder(
        robot_name="arm",
        package_name="arm_motion_planning",
    )
    .robot_description(
    file_path=os.path.join(arm_path, "urdf", "arm_cad.urdf.xacro"),
    mappings={
        "use_mock_hardware": "true",
    },
    )
    .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)

