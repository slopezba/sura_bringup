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


BASE_CONTROLLERS = [
    "thruster_test_controller",
    "body_force",
    "state_observer",
    "body_velocity",
    "stabilize",
    "depth_hold",
    "position_hold",
]

SENSOR_BROADCASTERS = [
    "imu_broadcaster",
    "magnetometer_broadcaster",
    "pressure_broadcaster",
    "dvl75_velocity_broadcaster",
    "dvl75_altitude_broadcaster",
    "dvl75_gps_broadcaster",
]

DUAL_ALPHA_CONTROLLERS = [
    "joint_state_broadcaster",
    "alpha_left_forward_velocity_controller",
    "alpha_right_forward_velocity_controller",
    "task_priority_controller",
]

SINGLE_ALPHA_CONTROLLERS = [
    "joint_state_broadcaster",
    "alpha_left_forward_velocity_controller",
]

ROBOT_VARIANT_TO_ARMS = {
    "dual_alpha": "dual",
    "single_alpha": "single",
    "auv": "auv",
}

PARAMS_BY_ARMS = {
    "dual": "ros2_control_params_dual_alpha.yaml",
    "single": "ros2_control_params_single_alpha.yaml",
    "auv": "ros2_control_params.yaml",
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


def discover_joint_controllers(robot_description_xml):
    root = ET.fromstring(robot_description_xml)
    controllers = []

    for ros2_control in root.findall("ros2_control"):
        for joint in ros2_control.findall("joint"):
            for param in joint.findall("param"):
                if param.attrib.get("name") == "controller" and param.text:
                    controllers.append(param.text.strip())

    return controllers


def prune_params_file(
    params_file,
    available_systems,
    available_broadcasters,
    available_joint_controllers,
):
    data = yaml.safe_load(Path(params_file).read_text(encoding="utf-8")) or {}
    controller_manager = data.setdefault("controller_manager", {}).setdefault(
        "ros__parameters", {}
    )

    hardware_states = controller_manager.get("hardware_components_initial_state", {})
    for state_name, component_names in hardware_states.items():
        if isinstance(component_names, list):
            hardware_states[state_name] = [
                component_name
                for component_name in component_names
                if component_name in available_systems
            ]

    fixed_controller_names = {
        "thruster_test_controller",
        "body_velocity_controller",
        "body_position_controller",
        "joint_state_broadcaster",
        "alpha_left_forward_velocity_controller",
        "alpha_right_forward_velocity_controller",
        "task_priority_controller",
    }

    removable_controllers = []
    for name, value in controller_manager.items():
        if not (isinstance(value, dict) and "type" in value):
            continue

        is_broadcaster = name.endswith("_broadcaster")
        is_optional_joint_controller = (
            name.endswith("_controller")
            and name not in fixed_controller_names
            and name not in available_joint_controllers
        )

        if (is_broadcaster and name not in available_broadcasters) or is_optional_joint_controller:
            removable_controllers.append(name)

    for name in removable_controllers:
        controller_manager.pop(name, None)
        data.pop(name, None)

    Path(params_file).write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def load_controller_types(params_file):
    data = yaml.safe_load(Path(params_file).read_text(encoding="utf-8")) or {}
    controller_manager = data.get("controller_manager", {}).get("ros__parameters", {})
    return [
        name
        for name, value in controller_manager.items()
        if isinstance(value, dict) and "type" in value
    ]


def description_namespace(description_package_name):
    return description_package_name.removesuffix("_description")


def resolve_arms(arms, robot_variant):
    if arms:
        if arms not in PARAMS_BY_ARMS:
            raise RuntimeError(
                "Unsupported arms '{}'. Use 'dual', 'single' or 'auv'.".format(arms)
            )
        return arms

    if robot_variant not in ROBOT_VARIANT_TO_ARMS:
        raise RuntimeError(
            "Unsupported robot_variant '{}'. Use 'dual_alpha', 'single_alpha' or 'auv'.".format(
                robot_variant
            )
        )
    return ROBOT_VARIANT_TO_ARMS[robot_variant]


def build_namespaced_params(
    template_file,
    robot_namespace,
    template_namespace,
    arms,
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

    output_file = f"/tmp/sura_bringup_{robot_namespace}_{arms}_ros2_control.yaml"
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
    description_package_name = LaunchConfiguration("robot_namespace_description").perform(
        context
    )
    robot_profile = load_robot_profile(description_package_name)
    description_profile = robot_profile.get("description", {})
    ros2_control_profile = robot_profile.get("ros2_control", {})
    robot_variant = LaunchConfiguration("robot_variant").perform(context)
    arms = resolve_arms(LaunchConfiguration("arms").perform(context), robot_variant)
    environment = LaunchConfiguration("environment").perform(context)

    if environment not in ("sim", "real"):
        raise RuntimeError(
            f"Unsupported environment '{environment}'. Use 'sim' or 'real'."
        )

    description_pkg = get_package_share_directory(description_package_name)
    hardware_pkg = get_package_share_directory("sura_hardware_interface")

    xacro_name = (
        "cirtesub_dual_alpha.urdf.xacro"
        if arms in ("dual", "single")
        else "cirtesub.urdf.xacro"
    )
    params_name = PARAMS_BY_ARMS[arms]

    csv_file = os.path.join(hardware_pkg, "config", "t500_lookup.csv")
    xacro_relative_path = description_profile.get("xacro")
    if xacro_relative_path:
        xacro_file = os.path.join(description_pkg, xacro_relative_path)
    else:
        xacro_file = os.path.join(description_pkg, "urdf", xacro_name)

    params_package = ros2_control_profile.get("params_package")
    params_relative_path = ros2_control_profile.get("params")
    if params_package and params_relative_path:
        template_file = os.path.join(
            get_package_share_directory(params_package),
            params_relative_path,
        )
    else:
        template_file = os.path.join(description_pkg, "config", params_name)

    params_file = build_namespaced_params(
        template_file,
        robot_namespace,
        description_namespace(description_package_name),
        arms,
    )

    xacro_args = dict(description_profile.get("xacro_args", {}))
    xacro_args["robot_namespace"] = robot_namespace
    xacro_args["environment"] = environment
    xacro_args.setdefault("lookup_csv", csv_file)
    if not xacro_args.get("lookup_csv"):
        xacro_args["lookup_csv"] = csv_file

    if not xacro_args.get("stonefish_topic"):
        xacro_args["stonefish_topic"] = f"/{robot_namespace}/controller/thruster_setpoints_sim"

    if not description_profile:
        if arms in ("dual", "single"):
            alpha_use_sim = "true" if environment == "sim" else "false"
            xacro_args.update(
                {
                    "arms": arms,
                    "use_sim": alpha_use_sim,
                    "alpha_use_fake_hardware": LaunchConfiguration("alpha_use_fake_hardware").perform(context),
                    "alpha_left_serial_port": LaunchConfiguration("alpha_left_serial_port").perform(context),
                    "alpha_right_serial_port": LaunchConfiguration("alpha_right_serial_port").perform(context),
                    "alpha_left_state_update_frequency": LaunchConfiguration(
                        "alpha_left_state_update_frequency"
                    ).perform(context),
                    "alpha_right_state_update_frequency": LaunchConfiguration(
                        "alpha_right_state_update_frequency"
                    ).perform(context),
                    "initial_positions_file": LaunchConfiguration("initial_positions_file").perform(context),
                    "alpha_desired_joint_states_topic": f"/{robot_namespace}/alpha/desired_joint_states",
                    "alpha_joint_states_topic": f"/{robot_namespace}/alpha/joint_states",
                }
            )

    xacro_command = build_xacro_command(xacro_file, xacro_args)
    robot_description_xml = render_robot_description(xacro_file, xacro_args)
    ros2_control_systems = discover_ros2_control_systems(robot_description_xml)
    sensor_broadcasters = discover_sensor_broadcasters(robot_description_xml)
    joint_controllers = discover_joint_controllers(robot_description_xml)
    prune_params_file(
        params_file,
        ros2_control_systems,
        sensor_broadcasters,
        joint_controllers,
    )
    configured_controller_types = load_controller_types(params_file)

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

    if not robot_profile and arms in ("dual", "single"):
        alpha_controllers = (
            DUAL_ALPHA_CONTROLLERS if arms == "dual" else SINGLE_ALPHA_CONTROLLERS
        )
        nodes.append(spawner("joint_state_broadcaster", controller_manager, inactive=False))
        nodes.extend(
            spawner(controller, controller_manager)
            for controller in alpha_controllers
            if controller != "joint_state_broadcaster"
        )

    if robot_profile:
        for controller in configured_controller_types:
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
    else:
        nodes.extend(spawner(controller, controller_manager) for controller in BASE_CONTROLLERS)
        nodes.extend(
            spawner(controller, controller_manager, inactive=False)
            for controller in (sensor_broadcasters or SENSOR_BROADCASTERS)
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="sura"),
            DeclareLaunchArgument(
                "robot_namespace_description",
                default_value="cirtesub_description",
            ),
            DeclareLaunchArgument("robot_variant", default_value="dual_alpha"),
            DeclareLaunchArgument("arms", default_value=""),
            DeclareLaunchArgument("environment", default_value="sim"),
            DeclareLaunchArgument("alpha_use_fake_hardware", default_value="true"),
            DeclareLaunchArgument("alpha_left_serial_port", default_value=""),
            DeclareLaunchArgument("alpha_right_serial_port", default_value=""),
            DeclareLaunchArgument("alpha_left_state_update_frequency", default_value="250"),
            DeclareLaunchArgument("alpha_right_state_update_frequency", default_value="250"),
            DeclareLaunchArgument("initial_positions_file", default_value="initial_positions.yaml"),
            OpaqueFunction(function=launch_setup),
        ]
    )
