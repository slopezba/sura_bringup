import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


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
    "leak_broadcaster",
]

AUV_ACTIVE_CONTROLLERS = [
    "lights_controller",
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

XACRO_BY_ARMS = {
    "dual": "cirtesub_dual_alpha.urdf.xacro",
    "single": "cirtesub_dual_alpha.urdf.xacro",
    "auv": None,
}


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


def find_auv_xacro(description_pkg, robot_namespace):
    candidates = [
        os.path.join(description_pkg, "urdf", robot_namespace, f"{robot_namespace}.urdf.xacro"),
        os.path.join(description_pkg, "urdf", f"{robot_namespace}.urdf.xacro"),
        os.path.join(description_pkg, "urdf", robot_namespace, "bluerov.urdf.xacro"),
        os.path.join(description_pkg, "urdf", "bluerov", "bluerov.urdf.xacro"),
        os.path.join(description_pkg, "urdf", "cirtesub.urdf.xacro"),
    ]

    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate

    raise RuntimeError(
        "Could not find AUV xacro file in '{}'. Tried: {}".format(
            description_pkg,
            ", ".join(candidates),
        )
    )


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
    description_package_name = f"{robot_namespace}_description"

    robot_variant = LaunchConfiguration("robot_variant").perform(context)
    arms = resolve_arms(LaunchConfiguration("arms").perform(context), robot_variant)
    environment = LaunchConfiguration("environment").perform(context)

    if environment not in ("sim", "real"):
        raise RuntimeError(
            f"Unsupported environment '{environment}'. Use 'sim' or 'real'."
        )

    description_pkg = get_package_share_directory(description_package_name)
    hardware_pkg = get_package_share_directory("sura_hardware_interface")

    if arms in ("dual", "single"):
        xacro_file = os.path.join(
            description_pkg,
            "urdf",
            XACRO_BY_ARMS[arms],
        )
        csv_name = "t500_lookup.csv"
        template_namespace = robot_namespace
    else:
        xacro_file = find_auv_xacro(description_pkg, robot_namespace)
        csv_name = "t200_lookup.csv"
        template_namespace = robot_namespace

    params_name = PARAMS_BY_ARMS[arms]

    csv_file = os.path.join(hardware_pkg, "config", csv_name)
    template_file = os.path.join(description_pkg, "config", params_name)

    params_file = build_namespaced_params(
        template_file,
        robot_namespace,
        template_namespace,
        arms,
    )

    xacro_command = [
        "xacro ",
        xacro_file,
        " robot_namespace:=",
        robot_namespace,
        " environment:=",
        environment,
        " lookup_csv:=",
        csv_file,
        " stonefish_topic:=/",
        robot_namespace,
        "/controller/thruster_setpoints_sim",
    ]

    if arms in ("dual", "single"):
        alpha_use_sim = "true" if environment == "sim" else "false"
        xacro_command.extend(
            [
                " arms:=",
                arms,
                " use_sim:=",
                alpha_use_sim,
                " alpha_use_fake_hardware:=",
                LaunchConfiguration("alpha_use_fake_hardware"),
                " alpha_left_serial_port:=",
                LaunchConfiguration("alpha_left_serial_port"),
                " alpha_right_serial_port:=",
                LaunchConfiguration("alpha_right_serial_port"),
                " alpha_left_state_update_frequency:=",
                LaunchConfiguration("alpha_left_state_update_frequency"),
                " alpha_right_state_update_frequency:=",
                LaunchConfiguration("alpha_right_state_update_frequency"),
                " initial_positions_file:=",
                LaunchConfiguration("initial_positions_file"),
                " alpha_desired_joint_states_topic:=/",
                robot_namespace,
                "/alpha/desired_joint_states",
                " alpha_joint_states_topic:=/",
                robot_namespace,
                "/alpha/joint_states",
            ]
        )

    robot_description = Command(xacro_command)

    controller_manager = f"/{robot_namespace}/controller/controller_manager"

    nodes = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=f"{robot_namespace}/controller",
            parameters=[
                params_file,
                {
                    "robot_description": robot_description,
                },
            ],
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

    if arms in ("dual", "single"):
        alpha_controllers = (
            DUAL_ALPHA_CONTROLLERS if arms == "dual" else SINGLE_ALPHA_CONTROLLERS
        )

        nodes.append(
            spawner(
                "joint_state_broadcaster",
                controller_manager,
                inactive=False,
            )
        )

        nodes.extend(
            spawner(controller, controller_manager)
            for controller in alpha_controllers
            if controller != "joint_state_broadcaster"
        )

    base_controllers = list(BASE_CONTROLLERS)

    nodes.extend(
        spawner(controller, controller_manager)
        for controller in base_controllers
    )

    if arms == "auv":
        nodes.extend(
            spawner(controller, controller_manager, inactive=False)
            for controller in AUV_ACTIVE_CONTROLLERS
        )

    nodes.extend(
        spawner(controller, controller_manager, inactive=False)
        for controller in SENSOR_BROADCASTERS
    )

    if environment == "real":
        nodes.append(
            spawner(
                "battery_broadcaster",
                controller_manager,
                inactive=False,
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="bluerov"),
            DeclareLaunchArgument("robot_variant", default_value="auv"),
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
