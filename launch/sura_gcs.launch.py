import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_bringup_description(robot_namespace):
    description_package = f"{robot_namespace}_description"
    config_path = os.path.join(
        get_package_share_directory(description_package),
        "config",
        "bringup_description.yaml",
    )
    with open(config_path, "r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    if not robot_namespace:
        raise RuntimeError("Launch argument 'robot_namespace' cannot be empty.")

    robot_profile = load_bringup_description(robot_namespace)
    robot_data = robot_profile.get("robot", {})
    profile_robot_name = str(robot_data.get("name", "")).strip("/")
    if profile_robot_name and profile_robot_name != robot_namespace:
        raise RuntimeError(
            f"robot.name '{profile_robot_name}' from bringup_description.yaml must match "
            f"launch argument robot_namespace '{robot_namespace}'."
        )

    environment = LaunchConfiguration("environment").perform(context).strip()
    if not environment:
        environment = str(robot_data.get("environment", "")).strip()
    if environment not in ("sim", "real"):
        raise RuntimeError(f"environment must be 'sim' or 'real', got '{environment}'.")

    description_profile = robot_profile.get("description", {})
    profile_description_package = str(description_profile.get("package", "")).strip()
    profile_xacro_file = str(description_profile.get("xacro", "")).strip()

    robot_description_package = (
        LaunchConfiguration("robot_description_package").perform(context).strip()
        or profile_description_package
        or f"{robot_namespace}_description"
    )
    robot_description_file = (
        LaunchConfiguration("robot_description_file").perform(context).strip()
        or profile_xacro_file
    )
    if not robot_description_file:
        raise RuntimeError("description.xacro must be defined in bringup_description.yaml.")

    xacro_arguments = LaunchConfiguration("xacro_arguments").perform(context).strip()
    if not xacro_arguments:
        xacro_arguments = f"robot_name:={robot_namespace} environment:={environment}"

    bringup_share = get_package_share_directory("sura_bringup")
    teleop_share = get_package_share_directory("sura_teleop")
    description_share = get_package_share_directory(robot_description_package)

    rviz_config_file = os.path.join(description_share, "config", f"{robot_namespace}.rviz")
    if not os.path.isfile(rviz_config_file):
        rviz_config_file = os.path.join(bringup_share, "config", "sura.rviz")

    teleop_launch_file = os.path.join(teleop_share, "launch", "teleop.launch.py")

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare(robot_description_package), robot_description_file]
                    ),
                    " ",
                    xacro_arguments,
                ]
            ),
            value_type=str,
        )
    }

    rviz_parameters = [
        robot_description,
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]

    semantic_package = LaunchConfiguration("robot_description_semantic_package").perform(context).strip()
    semantic_file = LaunchConfiguration("robot_description_semantic_file").perform(context).strip()
    if semantic_package and semantic_file:
        robot_description_semantic = {
            "robot_description_semantic": ParameterValue(
                Command(
                    [
                        PathJoinSubstitution([FindExecutable(name="xacro")]),
                        " ",
                        PathJoinSubstitution([FindPackageShare(semantic_package), semantic_file]),
                        " ",
                        "robot_name:=",
                        robot_namespace,
                    ]
                ),
                value_type=str,
            )
        }
        rviz_parameters.append(robot_description_semantic)

    moveit_dir = os.path.join(description_share, "moveit2")
    for moveit_file in (
        "planning_pipelines.yaml",
        "ompl_planning.yaml",
        "joint_limits.yaml",
        "kinematics.yaml",
    ):
        moveit_path = os.path.join(moveit_dir, moveit_file)
        if os.path.isfile(moveit_path):
            rviz_parameters.append(moveit_path)

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch_file),
        launch_arguments={
            "robot_namespace": robot_namespace,
            "environment": environment,
        }.items(),
        condition=IfCondition(LaunchConfiguration("teleop_enabled")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=rviz_parameters,
    )

    return [
        teleop_launch,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value=""),
        DeclareLaunchArgument("environment", default_value=""),
        DeclareLaunchArgument("robot_description_package", default_value=""),
        DeclareLaunchArgument("robot_description_file", default_value=""),
        DeclareLaunchArgument("robot_description_semantic_package", default_value=""),
        DeclareLaunchArgument("robot_description_semantic_file", default_value=""),
        DeclareLaunchArgument("xacro_arguments", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("teleop_enabled", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
