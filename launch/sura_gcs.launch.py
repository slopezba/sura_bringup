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


def load_bringup_description(description_share):
    config_path = os.path.join(description_share, "config", "bringup_description.yaml")
    if not os.path.exists(config_path):
        return {}

    with open(config_path, "r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def launch_setup(context, *args, **kwargs):
    teleop_share = get_package_share_directory("sura_teleop")
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    robot_description_package = LaunchConfiguration("robot_description_package").perform(context).strip()
    robot_description_file = LaunchConfiguration("robot_description_file").perform(context).strip()
    rviz_config_file = LaunchConfiguration("rviz_config_file").perform(context).strip()
    xacro_arguments = LaunchConfiguration("xacro_arguments").perform(context).strip()
    use_sim_time = LaunchConfiguration("use_sim_time")
    teleop_enabled = LaunchConfiguration("teleop_enabled")

    if not robot_namespace:
        raise RuntimeError("robot_namespace must be defined")

    if not robot_description_package:
        robot_description_package = f"{robot_namespace}_description"

    description_share = get_package_share_directory(robot_description_package)
    bringup_description = load_bringup_description(description_share)
    description_profile = bringup_description.get("description", {})

    if not robot_description_file:
        robot_description_file = str(description_profile.get("xacro", "")).strip()
    if not robot_description_file:
        robot_description_file = os.path.join("urdf", f"{robot_namespace}.urdf.xacro")

    if not rviz_config_file:
        rviz_config_file = os.path.join(description_share, "config", f"{robot_namespace}.rviz")

    if not xacro_arguments:
        environment = str(bringup_description.get("robot", {}).get("environment", "sim")).strip()
        xacro_arguments = f"robot_name:={robot_namespace} environment:={environment}"

    teleop_launch_file = os.path.join(teleop_share, "launch", "teleop.launch.py")
    robot_description_path = os.path.join(description_share, robot_description_file)

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    robot_description_path,
                    " ",
                    xacro_arguments,
                ]
            ),
            value_type=str,
        )
    }

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch_file),
        launch_arguments={"robot_namespace": robot_namespace}.items(),
        condition=IfCondition(teleop_enabled),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    return [teleop_launch, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value=""),
        DeclareLaunchArgument("robot_description_package", default_value=""),
        DeclareLaunchArgument("robot_description_file", default_value=""),
        DeclareLaunchArgument("rviz_config_file", default_value=""),
        DeclareLaunchArgument("xacro_arguments", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("teleop_enabled", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
