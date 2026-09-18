"""
move_group em modo APENAS PLANEJAMENTO.

Não sobe ros2_control nem controlador nenhum: nesta etapa o MoveIt
existe para responder "me dê uma trajetória", e quem reproduz a
trajetória é o robô 3D da interface web.

    interface web  ->  backend FastAPI  ->  moveit_bridge (HTTP 8081)
                                                  |
                                                  | /compute_ik
                                                  | /plan_kinematic_path
                                                  v
                                              move_group
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    urdf_xacro = os.path.join(
        get_package_share_directory("problemas2_urdf_description"),
        "urdf",
        "problemas2_urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "problemas2_urdf",
            package_name="problemas2_moveit_config",
        )
        .robot_description(file_path=urdf_xacro)
        .robot_description_semantic(file_path="config/problemas2.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Nesta etapa ninguém executa trajetória: o move_group planeja e a
    # interface web reproduz. O bridge é a única fonte de /joint_states.
    extra = {
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), extra],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    bridge = Node(
        package="moveit_bridge",
        executable="bridge_node",
        output="screen",
        parameters=[
            {"http_port": LaunchConfiguration("http_port")},
            {"planning_group": "arm"},
            {"tip_link": "tool0"},
            {"base_frame": "base_link"},
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory("problemas2_moveit_config"),
        "config",
        "moveit.rviz",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("http_port", default_value="8081"),
            DeclareLaunchArgument("rviz", default_value="false"),
            robot_state_publisher,
            move_group,
            bridge,
            rviz,
        ]
    )
