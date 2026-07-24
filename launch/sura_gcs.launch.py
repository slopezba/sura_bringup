import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory("sura_bringup")
    cirtesub_share = get_package_share_directory("cirtesub_description")
    teleop_share = get_package_share_directory("sura_teleop")
    robot_namespace = LaunchConfiguration("robot_namespace")
    robot_description_package = LaunchConfiguration("robot_description_package")
    robot_description_file = LaunchConfiguration("robot_description_file")
    robot_description_semantic_package = LaunchConfiguration("robot_description_semantic_package")
    robot_description_semantic_file = LaunchConfiguration("robot_description_semantic_file")
    xacro_arguments = LaunchConfiguration("xacro_arguments")
    use_sim_time = LaunchConfiguration("use_sim_time")
    teleop_enabled = LaunchConfiguration("teleop_enabled")

    rviz_config_file = os.path.join(bringup_share, "config", "sura.rviz")
    teleop_launch_file = os.path.join(teleop_share, "launch", "teleop.launch.py")
    planning_pipelines = os.path.join(cirtesub_share, "moveit2", "planning_pipelines.yaml")
    ompl_planning = os.path.join(cirtesub_share, "moveit2", "ompl_planning.yaml")
    joint_limits = os.path.join(cirtesub_share, "moveit2", "joint_limits.yaml")
    kinematics = os.path.join(cirtesub_share, "moveit2", "kinematics.yaml")

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

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare(robot_description_semantic_package),
                            robot_description_semantic_file,
                        ]
                    ),
                    " ",
                    "robot_name:=",
                    robot_namespace,
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
            robot_description_semantic,
            planning_pipelines,
            ompl_planning,
            joint_limits,
            kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="cirtesub"),
        DeclareLaunchArgument(
            "robot_description_package",
            default_value="cirtesub_description",
        ),
        DeclareLaunchArgument(
            "robot_description_file",
            default_value=os.path.join("urdf", "cirtesub_dual_alpha.urdf.xacro"),
        ),
        DeclareLaunchArgument(
            "robot_description_semantic_package",
            default_value="cirtesub_description",
        ),
        DeclareLaunchArgument(
            "robot_description_semantic_file",
            default_value=os.path.join("moveit2", "cirtesub_dual_alpha.srdf.xacro"),
        ),
        DeclareLaunchArgument(
            "xacro_arguments",
            default_value=(
                "robot_name:=cirtesub environment:=sim arms:=dual use_sim:=true "
                "alpha_desired_joint_states_topic:=/cirtesub/alpha/desired_joint_states "
                "alpha_joint_states_topic:=/cirtesub/stonefish/alpha/joint_states "
                "alpha_normalized_joint_states_topic:=/cirtesub/alpha/joint_states"
            ),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("teleop_enabled", default_value="true"),
        teleop_launch,
        rviz_node,
    ])
