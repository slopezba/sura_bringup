import os
import subprocess
import xml.etree.ElementTree as ET

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetRemap


CONTROLLER_GROUPS_BY_FAMILY = {
    "surface": ("common_controllers", "usv_controllers"),
    "underwater": ("common_controllers", "auv_controllers"),
}

DUAL_ALPHA_CONTROLLERS = [
    "joint_state_broadcaster",
    "alpha_left_forward_velocity_controller",
    "alpha_right_forward_velocity_controller",
    "alpha_left_cartesian_velocity_controller",
    "alpha_right_cartesian_velocity_controller",
    "alpha_left_joint_trajectory_controller",
    "alpha_right_joint_trajectory_controller",
    "task_priority_controller",
]

SINGLE_ALPHA_CONTROLLERS = [
    "joint_state_broadcaster",
    "alpha_left_forward_velocity_controller",
    "alpha_left_cartesian_velocity_controller",
    "alpha_left_joint_trajectory_controller",
]


def find_controller_manager_parameters(data):
    for node_name, node_params in data.items():
        if not isinstance(node_params, dict):
            continue
        if node_name.rstrip("/").endswith("controller_manager"):
            return node_params.get("ros__parameters", {})
    return data.get("controller_manager", {}).get("ros__parameters", {})


def discover_sensor_broadcasters(robot_description_root):
    broadcasters = []
    for ros2_control in robot_description_root.findall("ros2_control"):
        for sensor in ros2_control.findall("sensor"):
            for param in sensor.findall("param"):
                if param.attrib.get("name") == "broadcaster" and param.text:
                    broadcasters.append(param.text.strip())
    return broadcasters


def discover_joint_controllers(robot_description_root):
    controllers = []
    for ros2_control in robot_description_root.findall("ros2_control"):
        for joint in ros2_control.findall("joint"):
            for param in joint.findall("param"):
                if param.attrib.get("name") == "controller" and param.text:
                    controllers.append(param.text.strip())
    return controllers


def alpha_controllers_for_arms(arms):
    if arms == "dual":
        return DUAL_ALPHA_CONTROLLERS
    if arms == "single":
        return SINGLE_ALPHA_CONTROLLERS
    return []


def load_controllers_to_spawn(params_file, robot_family, arms, robot_description_root):
    with open(params_file, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    controller_manager = find_controller_manager_parameters(data)
    group_names = CONTROLLER_GROUPS_BY_FAMILY.get(robot_family)

    if group_names is None:
        supported = ", ".join(sorted(CONTROLLER_GROUPS_BY_FAMILY))
        raise RuntimeError(
            f"Unsupported robot family '{robot_family}'. Supported families: {supported}."
        )

    controllers = []
    for group_name in group_names:
        controllers.extend(controller_manager.get(group_name, []) or [])

    controllers.extend(discover_joint_controllers(robot_description_root))
    controllers.extend(discover_sensor_broadcasters(robot_description_root))
    controllers.extend(alpha_controllers_for_arms(arms))

    configured_controllers = {
        name
        for name, value in controller_manager.items()
        if isinstance(value, dict) and "type" in value
    }

    return [
        controller
        for controller in dict.fromkeys(controllers)
        if controller in configured_controllers
    ]


def spawner(controller_name, controller_manager, inactive=True):
    arguments = [controller_name, "--controller-manager", controller_manager]
    if inactive:
        arguments.append("--inactive")

    return Node(
        package="controller_manager",
        executable="spawner",
        name=f"spawn_{controller_name}",
        arguments=arguments,
        output="screen",
    )


def is_broadcaster(controller_name):
    return (
        controller_name == "joint_state_broadcaster"
        or controller_name.endswith("_broadcaster")
    )


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    robot_family = LaunchConfiguration("robot_family").perform(context).strip()
    arms = LaunchConfiguration("arms").perform(context).strip()
    description_package = LaunchConfiguration("description_package").perform(context).strip()
    xacro_file = LaunchConfiguration("xacro_file").perform(context).strip()
    xacro_arguments = LaunchConfiguration("xacro_arguments").perform(context).strip()
    params_package = LaunchConfiguration("ros2_control_params_package").perform(context).strip()
    params_file = LaunchConfiguration("ros2_control_params_file").perform(context).strip()

    description_path = os.path.join(get_package_share_directory(description_package), xacro_file)
    params_path = os.path.join(get_package_share_directory(params_package), params_file)

    xacro_command = ["xacro", description_path]
    xacro_command.extend(xacro_arguments.split())
    robot_description_root = ET.fromstring(subprocess.check_output(xacro_command, text=True))
    controllers_to_spawn = load_controllers_to_spawn(
        params_path,
        robot_family,
        arms,
        robot_description_root,
    )
    if not controllers_to_spawn:
        raise RuntimeError(f"No controllers to spawn for robot family '{robot_family}'.")

    controller_manager = f"/{robot_namespace}/controller/controller_manager"
    robot_description_command = Command(
        [
            "xacro ",
            description_path,
            " ",
            xacro_arguments,
        ]
    )

    joint_states_remap = SetRemap(
        src="/joint_states",
        dst=f"/{robot_namespace}/joint_states",
    )
    controller_joint_states_remap = SetRemap(
        src=f"/{robot_namespace}/controller/joint_states",
        dst=f"/{robot_namespace}/joint_states",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="controller",
        parameters=[
            params_path,
            {"robot_description": robot_description_command},
        ],
        output="screen",
    )

    broadcaster_spawners = [
        spawner(
            controller,
            controller_manager,
            inactive=False,
        )
        for controller in controllers_to_spawn
        if is_broadcaster(controller)
    ]

    controller_spawners = [
        spawner(
            controller,
            controller_manager,
            inactive=True,
        )
        for controller in controllers_to_spawn
        if not is_broadcaster(controller)
    ]

    return [
        GroupAction(
            [
                joint_states_remap,
                controller_joint_states_remap,
                ros2_control_node,
                *broadcaster_spawners,
                *controller_spawners,
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace"),
            DeclareLaunchArgument("robot_family"),
            DeclareLaunchArgument("arms", default_value="auv"),
            DeclareLaunchArgument("description_package"),
            DeclareLaunchArgument("xacro_file"),
            DeclareLaunchArgument("xacro_arguments"),
            DeclareLaunchArgument("ros2_control_params_package"),
            DeclareLaunchArgument("ros2_control_params_file"),
            OpaqueFunction(function=launch_setup),
        ]
    )
