import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import yaml


CONTROLLER_GROUPS_BY_FAMILY = {
    "surface": ("common_controllers", "usv_controllers"),
    "underwater": ("common_controllers", "auv_controllers"),
}

CONTROLLER_GROUP_NAMES = {
    group_name
    for group_names in CONTROLLER_GROUPS_BY_FAMILY.values()
    for group_name in group_names
}


def load_robot_profile(description_package_name):
    profile_file = os.path.join(
        get_package_share_directory(description_package_name),
        "config",
        "bringup_description.yaml",
    )
    if not os.path.exists(profile_file):
        return {}

    with open(profile_file, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def scalar_to_xacro_arg(value):
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def build_xacro_command(xacro_file, xacro_args):
    command = ["xacro ", xacro_file]
    for key, value in xacro_args.items():
        command.extend([" ", f"{key}:=", scalar_to_xacro_arg(value)])
    return command


def render_robot_description(xacro_file, xacro_args):
    command = ["xacro", xacro_file]
    command.extend(f"{key}:={scalar_to_xacro_arg(value)}" for key, value in xacro_args.items())
    return subprocess.check_output(command, text=True)


def discover_sensor_broadcasters(robot_description_xml):
    root = ET.fromstring(robot_description_xml)
    broadcasters = []

    for ros2_control in root.findall("ros2_control"):
        for sensor in ros2_control.findall("sensor"):
            for param in sensor.findall("param"):
                if param.attrib.get("name") == "broadcaster" and param.text:
                    broadcasters.append(param.text.strip())

    return broadcasters


def discover_ros2_control_systems(robot_description_xml):
    root = ET.fromstring(robot_description_xml)
    return [
        ros2_control.attrib["name"]
        for ros2_control in root.findall("ros2_control")
        if ros2_control.attrib.get("name")
    ]


def discover_robot_family(robot_description_xml, fallback=""):
    root = ET.fromstring(robot_description_xml)
    family = root.attrib.get("family", "").strip()
    if family:
        return family

    for param in root.findall("param"):
        if param.attrib.get("name") == "family" and param.text:
            return param.text.strip()

    return fallback


def discover_joint_controllers(robot_description_xml):
    root = ET.fromstring(robot_description_xml)
    controllers = []

    for ros2_control in root.findall("ros2_control"):
        for joint in ros2_control.findall("joint"):
            for param in joint.findall("param"):
                if param.attrib.get("name") == "controller" and param.text:
                    controllers.append(param.text.strip())

    return controllers


def load_selected_controller_groups(params_file, family):
    data = yaml.safe_load(Path(params_file).read_text(encoding="utf-8")) or {}
    controller_manager = find_controller_manager_parameters(data)
    group_names = CONTROLLER_GROUPS_BY_FAMILY.get(family)

    if group_names is None:
        supported = ", ".join(sorted(CONTROLLER_GROUPS_BY_FAMILY))
        raise RuntimeError(
            f"Unsupported robot family '{family}'. Supported families: {supported}."
        )

    controllers = []
    for group_name in group_names:
        group_controllers = controller_manager.get(group_name, [])
        if group_controllers is None:
            continue
        if not isinstance(group_controllers, list):
            raise RuntimeError(
                f"controller_manager.ros__parameters.{group_name} must be a list."
            )
        controllers.extend(str(controller) for controller in group_controllers)

    return controllers


def find_controller_manager_parameters(data):
    for node_name, node_params in data.items():
        if not isinstance(node_params, dict):
            continue
        if node_name.rstrip("/").endswith("controller_manager"):
            return node_params.setdefault("ros__parameters", {})
    return data.setdefault("controller_manager", {}).setdefault("ros__parameters", {})


def prune_params_file(
    params_file,
    available_systems,
    available_broadcasters,
    allowed_controllers,
):
    data = yaml.safe_load(Path(params_file).read_text(encoding="utf-8")) or {}
    controller_manager = find_controller_manager_parameters(data)

    for group_name in CONTROLLER_GROUP_NAMES:
        controller_manager.pop(group_name, None)

    hardware_states = controller_manager.get("hardware_components_initial_state", {})
    for state_name, component_names in hardware_states.items():
        if isinstance(component_names, list):
            hardware_states[state_name] = [
                component_name
                for component_name in component_names
                if component_name in available_systems
            ]

    removable_controllers = []
    for name, value in controller_manager.items():
        if not (isinstance(value, dict) and "type" in value):
            continue

        is_broadcaster = name.endswith("_broadcaster")
        is_disallowed_controller = name not in allowed_controllers

        if (is_broadcaster and name not in available_broadcasters) or is_disallowed_controller:
            removable_controllers.append(name)

    for name in removable_controllers:
        controller_manager.pop(name, None)
        data.pop(name, None)

    Path(params_file).write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def load_controller_types(params_file):
    data = yaml.safe_load(Path(params_file).read_text(encoding="utf-8")) or {}
    controller_manager = find_controller_manager_parameters(data)
    return [
        name
        for name, value in controller_manager.items()
        if isinstance(value, dict) and "type" in value
    ]


def description_namespace(description_package_name):
    return description_package_name.removesuffix("_description")


def build_namespaced_params(
    template_file,
    robot_namespace,
    template_namespace,
):
    text = Path(template_file).read_text(encoding="utf-8")
    text = text.replace(f"/{template_namespace}/", f"/{robot_namespace}/")
    text = text.replace(f"{template_namespace}/", f"{robot_namespace}/")
    text = text.replace(
        f"{template_namespace}_thrusters",
        f"{robot_namespace}_thrusters",
    )
    text = text.replace(
        f"/robot_state_publisher_{template_namespace}",
        f"/{robot_namespace}/robot_state_publisher",
    )

    output_file = "/tmp/ros2_control_params.yaml"
    Path(output_file).write_text(text, encoding="utf-8")
    return output_file


def spawner(controller_name, controller_manager, inactive=True):
    arguments = [controller_name, "--controller-manager", controller_manager]
    if inactive:
        arguments.append("--inactive")

    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=arguments,
        output="screen",
    )


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    description_package_name = (
        LaunchConfiguration("robot_namespace_description").perform(context).strip()
        or f"{robot_namespace}_description"
    )
    robot_profile = load_robot_profile(description_package_name)
    description_profile = robot_profile.get("description", {})
    ros2_control_profile = robot_profile.get("ros2_control", {})
    environment = LaunchConfiguration("environment").perform(context)

    if environment not in ("sim", "real"):
        raise RuntimeError(
            f"Unsupported environment '{environment}'. Use 'sim' or 'real'."
        )

    description_pkg = get_package_share_directory(description_package_name)
    hardware_pkg = get_package_share_directory("sura_hardware_interface")

    csv_file = os.path.join(hardware_pkg, "config", "t200_lookup.csv")
    xacro_relative_path = description_profile.get("xacro")
    if not xacro_relative_path:
        raise RuntimeError(
            f"description.xacro must be defined in {description_package_name}/config/bringup_description.yaml"
        )
    xacro_file = os.path.join(description_pkg, xacro_relative_path)

    params_package = ros2_control_profile.get("params_package")
    params_relative_path = ros2_control_profile.get("params")
    if params_package and params_relative_path:
        template_file = os.path.join(
            get_package_share_directory(params_package),
            params_relative_path,
        )
    else:
        template_file = os.path.join(description_pkg, "config", "ros2_control_params.yaml")

    params_file = build_namespaced_params(
        template_file,
        robot_namespace,
        description_namespace(description_package_name),
    )

    xacro_args = dict(description_profile.get("xacro_args", {}))
    xacro_args["robot_namespace"] = robot_namespace
    xacro_args["environment"] = environment
    xacro_args.setdefault("lookup_csv", csv_file)
    if not xacro_args.get("lookup_csv"):
        xacro_args["lookup_csv"] = csv_file

    if not xacro_args.get("stonefish_topic"):
        xacro_args["stonefish_topic"] = f"/{robot_namespace}/controller/thruster_setpoints_sim"

    xacro_command = build_xacro_command(xacro_file, xacro_args)
    robot_description_xml = render_robot_description(xacro_file, xacro_args)
    robot_family = discover_robot_family(
        robot_description_xml,
        str(robot_profile.get("robot", {}).get("family", "")),
    )
    ros2_control_systems = discover_ros2_control_systems(robot_description_xml)
    sensor_broadcasters = discover_sensor_broadcasters(robot_description_xml)
    joint_controllers = discover_joint_controllers(robot_description_xml)
    grouped_controllers = load_selected_controller_groups(params_file, robot_family)
    controllers_to_spawn = list(
        dict.fromkeys(grouped_controllers + joint_controllers + sensor_broadcasters)
    )
    prune_params_file(
        params_file,
        ros2_control_systems,
        sensor_broadcasters,
        set(controllers_to_spawn),
    )
    configured_controller_types = set(load_controller_types(params_file))
    controllers_to_spawn = [
        controller
        for controller in controllers_to_spawn
        if controller in configured_controller_types
    ]

    robot_description = Command(xacro_command)

    controller_manager = f"/{robot_namespace}/controller/controller_manager"
    nodes = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=f"{robot_namespace}/controller",
            parameters=[params_file, {"robot_description": robot_description}],
            remappings=[
                ("/joint_states", f"/{robot_namespace}/joint_states"),
                (
                    f"/{robot_namespace}/controller/joint_states",
                    f"/{robot_namespace}/joint_states",
                ),
            ],
            output="screen",
        )
    ]

    for controller in controllers_to_spawn:
        nodes.append(
            spawner(
                controller,
                controller_manager,
                inactive=not (
                    controller == "joint_state_broadcaster"
                    or controller.endswith("_broadcaster")
                ),
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="sura"),
            DeclareLaunchArgument(
                "robot_namespace_description",
                default_value="",
            ),
            DeclareLaunchArgument("environment", default_value="sim"),
            OpaqueFunction(function=launch_setup),
        ]
    )
