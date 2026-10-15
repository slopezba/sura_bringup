import copy
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


CONTROLLER_GROUPS_BY_FAMILY = {
    "surface": ("common_controllers", "usv_controllers"),
    "underwater": ("common_controllers", "auv_controllers"),
}

DUAL_ALPHA_CONTROLLERS = [
    "joint_state_broadcaster",
    "alpha_joint_state_broadcaster",
    "alpha_left_forward_velocity_controller",
    "alpha_right_forward_velocity_controller",
    "alpha_left_gripper_velocity_controller",
    "alpha_right_gripper_velocity_controller",
    "alpha_left_forward_position_controller",
    "alpha_right_forward_position_controller",
    "alpha_left_gripper_position_controller",
    "alpha_right_gripper_position_controller",
    "alpha_left_cartesian_velocity_controller",
    "alpha_right_cartesian_velocity_controller",
    "alpha_left_joint_trajectory_controller",
    "alpha_right_joint_trajectory_controller",
    "task_priority_controller",
]

SINGLE_ALPHA_CONTROLLERS = [
    "joint_state_broadcaster",
    "alpha_joint_state_broadcaster",
    "alpha_left_forward_velocity_controller",
    "alpha_left_gripper_velocity_controller",
    "alpha_left_forward_position_controller",
    "alpha_left_gripper_position_controller",
    "alpha_left_cartesian_velocity_controller",
    "alpha_left_joint_trajectory_controller",
]


def find_controller_manager_parameters(data):
    parameters = {}
    for node_name, node_params in data.items():
        if isinstance(node_params, dict) and node_name.rstrip("/").endswith("controller_manager"):
            parameters.update(node_params.get("ros__parameters", {}))
    return parameters


def split_robot_description(root):
    """Give each hardware component to exactly one manager; retain all kinematic links."""
    vehicle, alpha = copy.deepcopy(root), copy.deepcopy(root)
    for original, vehicle_component, alpha_component in zip(
        root.findall("ros2_control"), vehicle.findall("ros2_control"), alpha.findall("ros2_control")
    ):
        joint_names = ["/" + joint.get("name", "") for joint in original.findall("joint")]
        alpha_joints = [
            "/alpha_left/" in name or "/alpha_right/" in name for name in joint_names
        ]
        if any(alpha_joints) and not all(alpha_joints):
            raise RuntimeError("Cannot split a hardware component containing vehicle and Alpha joints.")
        if any(alpha_joints):
            vehicle.remove(vehicle_component)
        else:
            alpha.remove(alpha_component)
    return vehicle, alpha


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

    # Older robot profiles list plugin types without explicit controller groups.
    if not controllers:
        prefixes = tuple(f"sura_controllers/{group.removesuffix('_controllers')}/" for group in group_names)
        controllers.extend(
            name for name, value in controller_manager.items()
            if isinstance(value, dict) and str(value.get("type", "")).startswith(prefixes)
        )
    controllers.extend(discover_joint_controllers(robot_description_root))
    controllers.extend(discover_sensor_broadcasters(robot_description_root))
    controllers.extend(alpha_controllers_for_arms(arms))
    if robot_description_root.findall("ros2_control/joint"):
        controllers.append("joint_state_broadcaster")

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


def spawner(controller_name, controller_manager, inactive=True, load_only=False):
    arguments = [controller_name, "--controller-manager", controller_manager]
    if load_only:
        arguments.append("--load-only")
    elif inactive:
        arguments.append("--inactive")

    return Node(
        package="controller_manager",
        executable="spawner",
        name=f"spawn_{controller_name}",
        namespace=controller_manager.rsplit("/", 1)[0],
        arguments=arguments,
        output="screen",
    )


def is_broadcaster(controller_name):
    return (
        controller_name == "joint_state_broadcaster"
        or controller_name.endswith("_broadcaster")
    )


def chain_spawners(spawners):
    if not spawners:
        return []

    actions = [spawners[0]]
    actions.extend(
        RegisterEventHandler(
            OnProcessExit(
                target_action=current_spawner,
                on_exit=[next_spawner],
            )
        )
        for current_spawner, next_spawner in zip(spawners, spawners[1:])
    )
    return actions


def task_priority_remappings(robot_namespace, task_parameters):
    task_endpoints = ["output"]
    for task_id in task_parameters.get("task_ids", []):
        task_endpoints.extend(
            f"tasks/{task_id}/{suffix}" for suffix in
            ("state", "target", "feedforward", "joint_target", "joint_command",
             "enable_roll_pitch", "disable_roll_pitch", "joint_trajectory", "follow_joint_trajectory")
        )
    # Humble remaps the action's individual topics/services, not its base name.
    for task_id in task_parameters.get("task_ids", []):
        task_endpoints.extend(
            f"tasks/{task_id}/follow_joint_trajectory/_action/{suffix}"
            for suffix in ("send_goal", "get_result", "cancel_goal", "feedback", "status")
        )
    return [
        (f"/{robot_namespace}/alpha/controller/task_priority/{endpoint}",
         f"/{robot_namespace}/controller/task_priority/{endpoint}")
        for endpoint in task_endpoints
    ]


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

    vehicle_root, alpha_root = split_robot_description(robot_description_root)
    if alpha_root.findall("ros2_control") and arms not in ("single", "dual"):
        raise RuntimeError("Alpha hardware requires arms=single or arms=dual.")
    with open(params_path, "r", encoding="utf-8") as stream:
        configured = yaml.safe_load(stream) or {}

    # Supply the complete model to task priority directly: fetching parameters
    # synchronously from a controller-manager lifecycle callback can deadlock.
    task_key = f"/{robot_namespace}/alpha/controller/task_priority_controller"
    runtime_params_path = params_path
    cleanup_actions = []
    if task_key in configured:
        configured[task_key]["ros__parameters"]["robot_description"] = ET.tostring(
            robot_description_root, encoding="unicode"
        )
        with tempfile.NamedTemporaryFile(mode="w", prefix="sura_alpha_controllers_", suffix=".yaml", delete=False) as stream:
            yaml.safe_dump(configured, stream)
            runtime_params_path = stream.name

        def cleanup_parameters(context, *args, **kwargs):
            if os.path.exists(runtime_params_path):
                os.unlink(runtime_params_path)
            return []

        cleanup_actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=cleanup_parameters)
        ])))

    groups = []
    for manager_namespace, root, controller_arms in (
        ("controller", vehicle_root, "none"),
        ("alpha/controller", alpha_root, arms),
    ):
        if not root.findall("ros2_control"):
            continue
        manager = f"/{robot_namespace}/{manager_namespace}/controller_manager"
        if manager not in configured:
            raise RuntimeError(f"Missing controller manager configuration: {manager}")
        manager_parameters = configured[manager].get("ros__parameters", {})
        candidates = load_controllers_to_spawn(params_path, robot_family, controller_arms, root)
        controllers = [name for name in candidates if isinstance(manager_parameters.get(name), dict)]
        remappings = [
            ("/joint_states", f"/{robot_namespace}/joint_states"),
            (f"/{robot_namespace}/{manager_namespace}/joint_states", f"/{robot_namespace}/joint_states"),
            # Controllers subscribe to the full robot description for their kinematics;
            # each manager receives only its hardware partition as a parameter.
            ("~/robot_description", f"/{robot_namespace}/{manager_namespace}/hardware_description"),
        ]
        if manager_namespace == "alpha/controller":
            task_parameters = configured.get(
                f"/{robot_namespace}/alpha/controller/task_priority_controller", {}
            ).get("ros__parameters", {})
            remappings.extend(task_priority_remappings(robot_namespace, task_parameters))
        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=manager_namespace,
            parameters=[runtime_params_path, {"robot_description": ET.tostring(root, encoding="unicode")}],
            output="screen",
        )
        ordered = sorted(controllers, key=lambda name: not is_broadcaster(name))
        spawners = [
            spawner(name, manager, inactive=not is_broadcaster(name),
                    load_only=name == "task_priority_controller")
            for name in ordered
        ]
        groups.append(GroupAction([
            *[SetRemap(src=src, dst=dst) for src, dst in remappings],
            control_node,
            *chain_spawners(spawners),
        ]))
    return [*cleanup_actions, *groups]


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
