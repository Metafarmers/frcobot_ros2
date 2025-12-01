from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription


def generate_launch_description():
    """
    Generate launch description with all config files from:
    /config directory

    Config files loaded:
    - fairino5_v6_robot.urdf.xacro  : Robot URDF description
    - fairino5_v6_robot.srdf        : Semantic robot description (move groups, etc.)
    - kinematics.yaml               : IK solver configuration
    - joint_limits.yaml             : Joint velocity/acceleration limits
    - pilz_cartesian_limits.yaml    : Cartesian limits for Pilz planner
    - moveit_controllers.yaml       : Controller configuration
    - trajectory_execution.yaml     : Trajectory execution parameters
    """
    config_base_path = "config"
    robot_name = "fairino5_v6_robot"

    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name="fairino5_v6_moveit2_config")
        # Robot description (URDF)
        .robot_description(file_path=f"{config_base_path}/{robot_name}.urdf.xacro")
        # Semantic description (SRDF) - move groups, end effectors, etc.
        .robot_description_semantic(file_path=f"{config_base_path}/{robot_name}.srdf")
        # Kinematics solver configuration
        .robot_description_kinematics(file_path=f"{config_base_path}/kinematics.yaml")
        # Joint limits (velocity, acceleration)
        .joint_limits(file_path=f"{config_base_path}/joint_limits.yaml")
        # Pilz cartesian limits (for LIN, PTP, CIRC motions)
        .pilz_cartesian_limits(file_path=f"{config_base_path}/pilz_cartesian_limits.yaml")
        # Trajectory execution and controller config
        .trajectory_execution(file_path=f"{config_base_path}/moveit_controllers.yaml")
        # Planning scene monitor settings
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        # Planning pipelines - OMPL, CHOMP, and Pilz Industrial Motion Planner
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    return generate_demo_launch(moveit_config)
