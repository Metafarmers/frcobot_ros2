from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription

# def generate_launch_description():
#     moveit_config = (
#         MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
#         .robot_description(file_path="config/fairino5_v6_robot.urdf.xacro")
#         .robot_description_semantic(file_path="config/fairino5_v6_robot.srdf")
#         .trajectory_execution(file_path="config/trajectory_execution.yaml")
#         .planning_pipelines("ompl")
#         .to_moveit_configs()
#     )


#     return generate_demo_launch(moveit_config)


def generate_launch_description():

    moveit_config = (MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
    .robot_description(file_path="config/fairino5_v6_robot.urdf.xacro")
    .robot_description_semantic(file_path="config/fairino5_v6_robot.srdf")
    .trajectory_execution(file_path="config/trajectory_execution.yaml")
    .planning_pipelines("ompl")
    .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
