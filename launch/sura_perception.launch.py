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


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")

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

    if robot_namespace:
        return [
            GroupAction(
                [
                    PushRosNamespace(robot_namespace),
                    camera_launch,
                ]
            )
        ]

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
