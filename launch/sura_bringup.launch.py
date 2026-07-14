import os
import subprocess
import xml.etree.ElementTree as ET

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def load_bringup_description(robot_namespace):
    description_package = f"{robot_namespace}_description"
    config_path = os.path.join(
        get_package_share_directory(description_package),
        "config",
        "bringup_description.yaml",
    )
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def launch_value(value):
    return str(value).lower() if isinstance(value, bool) else str(value)


def normalize_description_arms(raw_arms, xacro_file):
    arms = str(raw_arms or "").strip().lower()
    if arms in ("dual_alpha", "dual"):
        return "dual"
    if arms in ("single_alpha", "single"):
        return "single"
    if arms in ("auv", "none", "no_alpha", "no_arms"):
        return "auv"
    if not arms:
        return "auv" if os.path.basename(xacro_file) == "cirtesub.urdf.xacro" else "dual"
    raise RuntimeError(
        "description.arms must be one of dual_alpha, dual, single_alpha, "
        f"single, auv or none; got '{raw_arms}'."
    )


def prepare_runtime_values(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    if not robot_namespace:
        raise RuntimeError("Launch argument 'robot_namespace' cannot be empty.")

    robot_profile = load_bringup_description(robot_namespace)

    robot_profile_data = robot_profile.get("robot", {})
    profile_robot_name = str(robot_profile_data.get("name", "")).strip("/")
    if profile_robot_name and profile_robot_name != robot_namespace:
        raise RuntimeError(
            f"robot.name '{profile_robot_name}' from bringup_description.yaml must match "
            f"launch argument robot_namespace '{robot_namespace}'."
        )

    environment = str(robot_profile_data.get("environment", "")).strip()
    if environment not in ("sim", "real"):
        raise RuntimeError(f"robot.environment must be 'sim' or 'real', got '{environment}'.")

    localization = str(robot_profile_data.get("localization", "")).strip()
    if localization not in ("sim", "real"):
        raise RuntimeError(f"robot.localization must be 'sim' or 'real', got '{localization}'.")

    description_profile = robot_profile.get("description", {})
    description_package = str(description_profile.get("package", "")).strip()
    if not description_package:
        raise RuntimeError("description.package must be defined in bringup_description.yaml")

    xacro_file = str(description_profile.get("xacro", "")).strip()
    if not xacro_file:
        raise RuntimeError("description.xacro must be defined in bringup_description.yaml")

    arms = normalize_description_arms(description_profile.get("arms", ""), xacro_file)
    xacro_arguments = f"robot_name:={robot_namespace} environment:={environment}"
    if arms in ("dual", "single"):
        use_sim = "true" if environment == "sim" else "false"
        xacro_arguments = (
            f"{xacro_arguments} arms:={arms} use_sim:={use_sim} "
            f"alpha_desired_joint_states_topic:=/{robot_namespace}/alpha/desired_joint_states "
            f"alpha_joint_states_topic:=/{robot_namespace}/alpha/joint_states"
        )

    xacro_command = [
        "xacro",
        os.path.join(get_package_share_directory(description_package), xacro_file),
    ]
    xacro_command.extend(xacro_arguments.split())
    robot_description_xml = subprocess.check_output(xacro_command, text=True)
    robot_description_root = ET.fromstring(robot_description_xml)

    robot_description_name = robot_description_root.attrib.get("name", "").strip("/")
    if robot_description_name != robot_namespace:
        raise RuntimeError(
            f"The rendered robot xacro name '{robot_description_name}' must match "
            f"robot.name '{robot_namespace}' from bringup_description.yaml."
        )

    robot_family = robot_description_root.attrib.get("family", "").strip()
    if not robot_family:
        raise RuntimeError("The rendered robot xacro must define robot family.")

    ros2_control_profile = robot_profile.get("ros2_control", {})
    diagnostics_profile = robot_profile.get("diagnostics", {})
    imu_profile = robot_profile.get("imu", {})
    localization_profile = robot_profile.get("localization", {})
    use_sim_localization = environment == "sim" and localization == "sim"
    use_real_localization = localization == "real"

    localization_launch_package = str(localization_profile.get("launch_package", "")).strip()
    if not localization_launch_package:
        raise RuntimeError("localization.launch_package must be defined in bringup_description.yaml")

    localization_launch_file = str(localization_profile.get("launch_file", "")).strip()
    if not localization_launch_file:
        raise RuntimeError("localization.launch_file must be defined in bringup_description.yaml")

    if "publish_tf" not in localization_profile:
        raise RuntimeError("localization.publish_tf must be defined in bringup_description.yaml")
    localization_publish_tf = localization_profile["publish_tf"]

    datum_profile = localization_profile.get("datum", {})
    if datum_profile is None:
        datum_profile = {}
    if not isinstance(datum_profile, dict):
        raise RuntimeError("localization.datum must be a map in bringup_description.yaml")

    datum_latitude = datum_profile.get("latitude", 0.0)
    datum_longitude = datum_profile.get("longitude", 0.0)
    datum_heading = datum_profile.get("heading", 0.0)

    raw_imu_topic = str(imu_profile.get("raw_imu_topic", "")).strip()
    if not raw_imu_topic:
        raise RuntimeError("imu.raw_imu_topic must be defined in bringup_description.yaml")

    filtered_imu_topic = str(imu_profile.get("filtered_imu_topic", "sensors/imu")).strip()
    if not filtered_imu_topic:
        raise RuntimeError("imu.filtered_imu_topic cannot be empty in bringup_description.yaml")

    mag_topic = str(imu_profile.get("mag_topic", "")).strip()
    if not mag_topic:
        raise RuntimeError("imu.mag_topic must be defined in bringup_description.yaml")

    params_package = ros2_control_profile.get("params_package", description_package)
    params_file = ros2_control_profile.get("params", "config/ros2_control_params.yaml")
    diagnostics_params_package = diagnostics_profile.get("params_package", "sura_diagnostics")
    diagnostics_params_file = diagnostics_profile.get("params", "config/diagnostics.yaml")
    cameras_profile = robot_profile.get("cameras", {})
    if cameras_profile is None:
        cameras_profile = {}
    if not isinstance(cameras_profile, dict):
        raise RuntimeError("cameras must be a map in bringup_description.yaml")

    launch_values = {
        "robot_namespace": robot_namespace,
        "environment": environment,
        "localization": localization,
        "robot_family": robot_family,
        "description_package": description_package,
        "xacro_file": xacro_file,
        "xacro_arguments": xacro_arguments,
        "arms": arms,
        "ros2_control_params_package": params_package,
        "ros2_control_params_file": params_file,
        "diagnostics_params_package": diagnostics_params_package,
        "diagnostics_params_file": diagnostics_params_file,
        "cameras": yaml.safe_dump(cameras_profile, default_flow_style=True),
        "raw_imu_topic": raw_imu_topic,
        "filtered_imu_topic": filtered_imu_topic,
        "mag_topic": mag_topic,
        "environment_is_real": environment == "real",
        "use_sim_localization": use_sim_localization,
        "use_real_localization": use_real_localization,
        "localization_launch_package": localization_launch_package,
        "localization_launch_file": localization_launch_file,
        "localization_publish_tf": localization_publish_tf,
        "localization_datum_latitude": datum_latitude,
        "localization_datum_longitude": datum_longitude,
        "localization_datum_heading": datum_heading,
    }

    return [
        SetLaunchConfiguration(name, launch_value(value))
        for name, value in launch_values.items()
    ]


def generate_launch_description():
    robot_namespace = LaunchConfiguration("robot_namespace")
    raw_imu_topic = LaunchConfiguration("raw_imu_topic")
    filtered_imu_topic = LaunchConfiguration("filtered_imu_topic")
    mag_topic = LaunchConfiguration("mag_topic")

    runtime_values = OpaqueFunction(function=prepare_runtime_values)

    robot_description_launch = GroupAction(
        [
            PushRosNamespace(robot_namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare(LaunchConfiguration("description_package")),
                            "launch",
                            "robot_description.launch.py",
                        ]
                    )
                ),
                launch_arguments=[
                    ("robot_description_package", LaunchConfiguration("description_package")),
                    ("xacro_file", LaunchConfiguration("xacro_file")),
                    ("xacro_arguments", LaunchConfiguration("xacro_arguments")),
                ],
            ),
        ]
    )

    ros2_control_launch = GroupAction(
        [
            PushRosNamespace(robot_namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("sura_bringup"),
                            "launch",
                            "sura_controllers.launch.py",
                        ]
                    )
                ),
                launch_arguments=[
                    ("robot_namespace", robot_namespace),
                    ("robot_family", LaunchConfiguration("robot_family")),
                    ("description_package", LaunchConfiguration("description_package")),
                    ("xacro_file", LaunchConfiguration("xacro_file")),
                    ("xacro_arguments", LaunchConfiguration("xacro_arguments")),
                    ("arms", LaunchConfiguration("arms")),
                    (
                        "ros2_control_params_package",
                        LaunchConfiguration("ros2_control_params_package"),
                    ),
                    ("ros2_control_params_file", LaunchConfiguration("ros2_control_params_file")),
                ],
            ),
        ]
    )

    # cameras_launch = GroupAction(
    #     [
    #         PushRosNamespace(robot_namespace),
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 PathJoinSubstitution(
    #                     [FindPackageShare("sura_cameras"), "launch", "cameras.launch.py"]
    #                 )
    #             ),
    #             launch_arguments=[
    #                 ("robot_namespace", robot_namespace),
    #                 ("environment", LaunchConfiguration("environment")),
    #                 ("cameras", LaunchConfiguration("cameras")),
    #             ],
    #         ),
    #     ],
    # )

    imu_launch = GroupAction(
        [
            PushRosNamespace(robot_namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("sura_imu"), "launch", "imu.launch.py"])
                ),
                launch_arguments=[
                    ("environment", LaunchConfiguration("environment")),
                    ("raw_imu_topic", raw_imu_topic),
                    ("mag_topic", mag_topic),
                    ("filtered_imu_topic", filtered_imu_topic),
                ],
            ),
        ],
    )

    sim_tf_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration("use_sim_localization")),
        actions=[
            PushRosNamespace(robot_namespace),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("cirtesu_tank_aruco_localization"),
                            "launch",
                            "aruco_map_localization.launch.py",
                        ]
                    )
                ),
                launch_arguments=[
                    ("robot_namespace", robot_namespace),
                ],
            ),
        ],
    )

    real_localization_launch = GroupAction(
        [
            PushRosNamespace(robot_namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare(LaunchConfiguration("localization_launch_package")),
                            "launch",
                            LaunchConfiguration("localization_launch_file"),
                        ]
                    )
                ),
                launch_arguments=[
                    ("robot_namespace", robot_namespace),
                    ("publish_tf", LaunchConfiguration("localization_publish_tf")),
                    ("datum_latitude", LaunchConfiguration("localization_datum_latitude")),
                    ("datum_longitude", LaunchConfiguration("localization_datum_longitude")),
                    ("datum_heading", LaunchConfiguration("localization_datum_heading")),
                ],
            ),
        ],
        condition=IfCondition(LaunchConfiguration("use_real_localization")),
    )

    thruster_force_publisher = GroupAction(
        [
            PushRosNamespace(robot_namespace),
            Node(
                package="sura_hardware_interface",
                executable="thruster_force_publisher",
                name="thruster_force_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_namespace": robot_namespace,
                        "robot_description_topic": ParameterValue(
                            ["/", robot_namespace, "/robot_description"],
                            value_type=str,
                        ),
                        "input_topic": ParameterValue(
                            ["/", robot_namespace, "/controller/body_force/output"],
                            value_type=str,
                        ),
                        "output_topic": ParameterValue(
                            ["/", robot_namespace, "/controller/thruster_forces"],
                            value_type=str,
                        ),
                    }
                ],
            ),
        ]
    )

    navigator_launch = GroupAction(
        [
            PushRosNamespace(robot_namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("sura_navigator"), "launch", "navigator.launch.py"]
                    )
                ),
                launch_arguments=[
                    ("robot_namespace", robot_namespace),
                    ("environment", LaunchConfiguration("environment")),
                    ("localization", LaunchConfiguration("localization")),
                    ("publish_tf", LaunchConfiguration("localization_publish_tf")),
                ],
            ),
        ]
    )

    diagnostics_launch = GroupAction(
        [
            PushRosNamespace(robot_namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("sura_diagnostics"), "launch", "diagnostics.launch.py"]
                    )
                ),
                launch_arguments=[
                    ("robot_namespace", robot_namespace),
                    (
                        "diagnostics_params_package",
                        LaunchConfiguration("diagnostics_params_package"),
                    ),
                    (
                        "diagnostics_params_file",
                        LaunchConfiguration("diagnostics_params_file"),
                    ),
                ],
            ),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value=""),
            runtime_values,
            robot_description_launch,
            ros2_control_launch,
            # cameras_launch,
            imu_launch,
            sim_tf_launch,
            thruster_force_publisher,
            real_localization_launch,
            navigator_launch,
            diagnostics_launch,
        ]
    )