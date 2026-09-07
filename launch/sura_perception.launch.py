import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def load_bringup_description(robot_namespace):
    description_package = f"{robot_namespace}_description"
    config_path = os.path.join(
        get_package_share_directory(description_package),
        "config",
        "bringup_description.yaml",
    )
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")

    if robot_namespace:
        robot_profile = load_bringup_description(robot_namespace)
        robot_data = robot_profile.get("robot", {})
        environment = str(robot_data.get("environment", "real")).strip() or "real"
        cameras = robot_profile.get("cameras", {})
        if cameras is None:
            cameras = {}
        if not isinstance(cameras, dict):
            raise RuntimeError("cameras must be a map in bringup_description.yaml")

        cameras_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("sura_cameras"),
                        "launch",
                        "cameras.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("environment", environment),
                ("cameras", yaml.safe_dump(cameras, default_flow_style=True)),
            ],
        )

        return [
            GroupAction(
                [
                    PushRosNamespace(robot_namespace),
                    cameras_launch,
                ]
            )
        ]

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("sura_cameras"),
                    "launch",
                    "single_camera.launch.py",
                ]
            )
        ),
        launch_arguments=[
            ("camera_name", LaunchConfiguration("camera_name")),
            ("camera_config_dir", LaunchConfiguration("camera_config_dir")),
            ("environment", LaunchConfiguration("environment")),
            ("aruco", LaunchConfiguration("aruco")),
        ],
    )

    return [camera_launch]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_name", default_value="down_camera"),
            DeclareLaunchArgument(
                "camera_config_dir",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("sura_cameras"),
                        "config",
                        "down_camera",
                    ]
                ),
            ),
            DeclareLaunchArgument("environment", default_value="real"),
            DeclareLaunchArgument("aruco", default_value="true"),
            DeclareLaunchArgument("robot_namespace", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
