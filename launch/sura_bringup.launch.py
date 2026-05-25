import os
import subprocess
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_description_package(robot_namespace):
    return f"{robot_namespace}_description"


def load_robot_profile(description_package_name):
    description_share = get_package_share_directory(description_package_name)
    profile_file = os.path.join(description_share, "config", "bringup_description.yaml")

    if not os.path.exists(profile_file):
        raise RuntimeError(
            f"Robot profile not found: {profile_file}. "
            "Expected config/bringup_description.yaml inside the description package."
        )

    with open(profile_file, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def include_launch(package_name, launch_name, launch_arguments):
    launch_file = os.path.join(
        get_package_share_directory(package_name),
        "launch",
        launch_name,
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=launch_arguments.items(),
    )


def build_xacro_arguments(xacro_args):
    return " ".join(
        f"{key}:={str(value).lower() if isinstance(value, bool) else value}"
        for key, value in xacro_args.items()
    )


def render_robot_description(description_package_name, xacro_relative_path, xacro_args):
    xacro_file = os.path.join(
        get_package_share_directory(description_package_name),
        xacro_relative_path,
    )
    command = ["xacro", xacro_file]
    command.extend(
        f"{key}:={str(value).lower() if isinstance(value, bool) else value}"
        for key, value in xacro_args.items()
    )
    return subprocess.check_output(command, text=True)


def xacro_contains_camera_actuator(robot_description_xml):
    root = ET.fromstring(robot_description_xml)
    for ros2_control in root.findall("ros2_control"):
        for element in ros2_control.iter():
            name = element.attrib.get("name", "")
            if "camera" in name.lower():
                return True
    return False


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace")
    robot_namespace_value = robot_namespace.perform(context).strip("/")

    robot_namespace_description_value = resolve_description_package(robot_namespace_value)
    robot_profile = load_robot_profile(robot_namespace_description_value)
    description_profile = robot_profile.get("description", {})
    xacro_args = dict(description_profile.get("xacro_args", {}))
    xacro_args["robot_namespace"] = robot_namespace_value
    xacro_args["environment"] = LaunchConfiguration("environment").perform(context)
    xacro_arguments = build_xacro_arguments(xacro_args)
    xacro_relative_path = description_profile.get("xacro", "")

    if not xacro_relative_path:
        raise RuntimeError("description.xacro must be defined in bringup_description.yaml")

    robot_description_xml = render_robot_description(
        robot_namespace_description_value,
        xacro_relative_path,
        xacro_args,
    )
    launch_cameras = xacro_contains_camera_actuator(robot_description_xml)

    environment = LaunchConfiguration("environment")
    localization = LaunchConfiguration("localization")

    environment_value = environment.perform(context)
    localization_value = localization.perform(context)

    common_arguments = {
        "robot_namespace": robot_namespace,
        "robot_namespace_description": robot_namespace_description_value,
        "environment": environment,
    }

    description_launch_arguments = {
        **common_arguments,
        "xacro_file": xacro_relative_path,
        "xacro_arguments": xacro_arguments,
    }

    launch_entities = [
        include_launch(
            robot_namespace_description_value,
            description_profile.get("launch_file", "robot_description.launch.py"),
            description_launch_arguments,
        ),
    ]

    ros2_control_profile = robot_profile.get("ros2_control", {})
    if ros2_control_profile.get("enabled", True):
        launch_entities.append(
            include_launch(
                ros2_control_profile.get("launch_package", "sura_bringup"),
                ros2_control_profile.get("launch_file", "sura_controllers.launch.py"),
                common_arguments,
            )
        )

    if launch_cameras:
        launch_entities.append(
            include_launch(
                "sura_cameras",
                "cameras.launch.py",
                {},
            )
        )

    def topic(path):
        return f"/{robot_namespace_value}/{path}"

    if environment_value == "real":
        launch_entities.append(
            include_launch(
                "sura_imu",
                "imu.launch.py",
                {
                    "raw_imu_topic": topic("controller/imu_broadcaster/imu"),
                    "mag_topic": topic("magnetometer_broadcaster/mag"),
                    "output_imu_topic": topic("sensors/imu"),
                    "calibrated_imu_topic": topic("imu/data_raw_calibrated"),
                    "calibrated_mag_topic": topic("imu/mag_calibrated"),
                    "base_frame": f"{robot_namespace_value}/base_link",
                    "imu_frame": f"{robot_namespace_value}/IMU",
                },
            )
        )

    use_sim_localization = environment_value == "sim" and localization_value == "sim"

    if use_sim_localization:
        launch_entities.extend(
            [
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="world_ned_to_world_enu",
                    output="screen",
                    arguments=[
                        "--x", "0.0",
                        "--y", "0.0",
                        "--z", "0.0",
                        "--roll", "3.14159265359",
                        "--pitch", "0.0",
                        "--yaw", "1.57079632679",
                        "--frame-id", "world_ned",
                        "--child-frame-id", "world_enu",
                    ],
                ),
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="world_ned_to_cirtesu_tank",
                    output="screen",
                    arguments=[
                        "--x", "0.0",
                        "--y", "0.0",
                        "--z", "0.0",
                        "--roll", "0.0",
                        "--pitch", "0.0",
                        "--yaw", "3.1416",
                        "--frame-id", "world_ned",
                        "--child-frame-id", "cirtesu_tank",
                    ],
                ),
                include_launch(
                    "cirtesu_tank_aruco_localization",
                    "aruco_map_localization.launch.py",
                    {},
                ),
            ]
        )
    else:
        localization_profile = robot_profile.get("localization", {})
        localization_enabled = localization_profile.get("enabled", True)

        if localization_enabled:
            localization_launch_package = localization_profile.get(
                "launch_package",
                "sura_localization",
            )
            localization_launch_file = localization_profile.get(
                "launch_file",
                "auv_localization.launch.py",
            )

            launch_entities.append(
                include_launch(
                    localization_launch_package,
                    localization_launch_file,
                    {
                        key: value
                        for key, value in {
                            "robot_namespace": robot_namespace,
                            "publish_tf": (
                                str(localization_profile["publish_tf"]).lower()
                                if "publish_tf" in localization_profile
                                else None
                            ),
                        }.items()
                        if value is not None
                    },
                )
            )

    launch_entities.extend(
        [
            include_launch(
                "sura_navigator",
                "navigator.launch.py",
                {
                    "robot_namespace": robot_namespace,
                    "environment": environment,
                    "localization": localization,
                },
            ),
            include_launch(
                "sura_diagnostics",
                "diagnostics.launch.py",
                {
                    "robot_namespace": robot_namespace,
                    "robot_variant": str(robot_profile.get("robot", {}).get("family", "")),
                },
            ),
        ]
    )

    teleop_profile = robot_profile.get("teleop", {})
    if teleop_profile.get("enabled", False):
        teleop_arguments = {
            "robot_namespace": robot_namespace,
        }
        if "robot" in teleop_profile:
            teleop_arguments["robot_model"] = str(teleop_profile["robot"])
        elif "profile" in teleop_profile:
            teleop_arguments["robot_model"] = str(teleop_profile["profile"])
        if "config" in teleop_profile:
            teleop_arguments["config_file"] = str(teleop_profile["config"])

        launch_entities.append(
            include_launch(
                teleop_profile.get("launch_package", "sura_teleop"),
                teleop_profile.get("launch_file", "teleop.launch.py"),
                teleop_arguments,
            )
        )

    return launch_entities


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="sura"),
            DeclareLaunchArgument("environment", default_value="sim"),
            DeclareLaunchArgument("localization", default_value="real"),
            OpaqueFunction(function=launch_setup),
        ]
    )
