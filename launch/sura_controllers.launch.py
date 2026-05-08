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
]

DUAL_ALPHA_CONTROLLERS = [
    "joint_state_broadcaster",
    "alpha_left_forward_velocity_controller",
    "alpha_right_forward_velocity_controller",
    "task_priority_controller",
]


def build_namespaced_params(template_file, robot_namespace, robot_variant):
    text = Path(template_file).read_text(encoding="utf-8")
    text = text.replace("/cirtesub/", f"/{robot_namespace}/")
    text = text.replace("cirtesub/", f"{robot_namespace}/")
    text = text.replace("cirtesub_thrusters", f"{robot_namespace}_thrusters")
    text = text.replace(
        "/robot_state_publisher_cirtesub",
        f"/{robot_namespace}/robot_state_publisher",
    )

    if robot_variant == "auv":
        text = text.replace(f"        - {robot_namespace}/alpha_left\n", "")
        text = text.replace(f"        - {robot_namespace}/alpha_right\n", "")

    output_file = f"/tmp/sura_bringup_{robot_namespace}_{robot_variant}_ros2_control.yaml"
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
    robot_variant = LaunchConfiguration("robot_variant").perform(context)
    environment = LaunchConfiguration("environment").perform(context)

    if robot_variant not in ("dual_alpha", "auv"):
        raise RuntimeError(
            f"Unsupported robot_variant '{robot_variant}'. Use 'dual_alpha' or 'auv'."
        )

    if environment not in ("sim", "real"):
        raise RuntimeError(
            f"Unsupported environment '{environment}'. Use 'sim' or 'real'."
        )

    description_pkg = get_package_share_directory("cirtesub_description")
    hardware_pkg = get_package_share_directory("sura_hardware_interface")
    template_pkg = get_package_share_directory("cirtesub_bringup")

    xacro_name = (
        "cirtesub_dual_alpha.urdf.xacro"
        if robot_variant == "dual_alpha"
        else "cirtesub.urdf.xacro"
    )
    params_name = (
        "ros2_control_params_dual_alpha.yaml"
        if robot_variant == "dual_alpha"
        else "ros2_control_params.yaml"
    )

    xacro_file = os.path.join(description_pkg, "urdf", xacro_name)
    csv_file = os.path.join(hardware_pkg, "config", "t500_lookup.csv")
    template_file = os.path.join(template_pkg, "config", params_name)
    params_file = build_namespaced_params(template_file, robot_namespace, robot_variant)

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

    if robot_variant == "dual_alpha":
        alpha_use_sim = "true" if environment == "sim" else "false"
        xacro_command.extend(
            [
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

    if robot_variant == "dual_alpha":
        nodes.append(spawner("joint_state_broadcaster", controller_manager, inactive=False))
        nodes.extend(
            spawner(controller, controller_manager)
            for controller in DUAL_ALPHA_CONTROLLERS
            if controller != "joint_state_broadcaster"
        )

    nodes.extend(spawner(controller, controller_manager) for controller in BASE_CONTROLLERS)
    nodes.extend(
        spawner(controller, controller_manager, inactive=False)
        for controller in SENSOR_BROADCASTERS
    )
    if environment == "real":
        nodes.append(spawner("battery_broadcaster", controller_manager, inactive=False))
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="sura"),
            DeclareLaunchArgument("robot_variant", default_value="dual_alpha"),
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
