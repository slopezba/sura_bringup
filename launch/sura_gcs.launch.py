import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("sura_bringup")
    teleop_share = get_package_share_directory("sura_teleop")

    rviz_config_file = os.path.join(bringup_share, "config", "cirtesub.rviz")
    teleop_launch_file = os.path.join(teleop_share, "launch", "teleop.launch.py")

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch_file)
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([teleop_launch, rviz_node])
