import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace")
    robot_namespace_value = robot_namespace.perform(context).strip("/")

    robot_description_package_value = f"{robot_namespace_value}_description"

    robot_variant = LaunchConfiguration("robot_variant")
    arms = LaunchConfiguration("arms")
    environment = LaunchConfiguration("environment")
    localization = LaunchConfiguration("localization")
    use_imu_calibration = LaunchConfiguration("use_imu_calibration")

    environment_value = environment.perform(context)
    localization_value = localization.perform(context)

    alpha_use_fake_hardware = LaunchConfiguration("alpha_use_fake_hardware")
    alpha_left_serial_port = LaunchConfiguration("alpha_left_serial_port")
    alpha_right_serial_port = LaunchConfiguration("alpha_right_serial_port")
    alpha_left_state_update_frequency = LaunchConfiguration(
        "alpha_left_state_update_frequency"
    )
    alpha_right_state_update_frequency = LaunchConfiguration(
        "alpha_right_state_update_frequency"
    )
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    pressure_offset_pa = LaunchConfiguration("pressure_offset_pa")

    common_arguments = {
        "robot_namespace": robot_namespace,
        "robot_variant": robot_variant,
        "arms": arms,
        "environment": environment,
        "alpha_use_fake_hardware": alpha_use_fake_hardware,
        "alpha_left_serial_port": alpha_left_serial_port,
        "alpha_right_serial_port": alpha_right_serial_port,
        "alpha_left_state_update_frequency": alpha_left_state_update_frequency,
        "alpha_right_state_update_frequency": alpha_right_state_update_frequency,
        "initial_positions_file": initial_positions_file,
    }

    launch_entities = [
        include_launch(
            robot_description_package_value,
            "robot_description.launch.py",
            common_arguments,
        ),
        include_launch(
            "sura_bringup",
            "sura_controllers.launch.py",
            common_arguments,
        ),
    ]

    if environment_value == "real":
        launch_entities.append(
            TimerAction(
                period=15.0,
                actions=[
                    include_launch(
                        "sura_imu",
                        "imu.launch.py",
                        {
                            "robot_namespace": robot_namespace,
                            "use_calibration": use_imu_calibration,
                        },
                    )
                ],
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
                    {
                        "robot_namespace": robot_namespace,
                    },
                ),
            ]
        )
    else:
        launch_entities.append(
            TimerAction(
                period=20.0,
                actions=[
                    include_launch(
                        "sura_localization",
                        "auv_localization.launch.py",
                        {
                            "robot_namespace": robot_namespace,
                            "environment": environment,
                            "pressure_offset_pa": pressure_offset_pa,
                        },
                    )
                ],
            )
        )

    launch_entities.extend(
        [
            TimerAction(
                period=25.0,
                actions=[
                    include_launch(
                        "sura_navigator",
                        "navigator.launch.py",
                        {
                            "robot_namespace": robot_namespace,
                            "environment": environment,
                            "localization": localization,
                        },
                    )
                ],
            ),
            TimerAction(
                period=30.0,
                actions=[
                    include_launch(
                        "sura_diagnostics",
                        "diagnostics.launch.py",
                        {
                            "robot_namespace": robot_namespace,
                            "robot_variant": robot_variant,
                        },
                    )
                ],
            ),
            TimerAction(
                period=35.0,
                actions=[
                    include_launch(
                        "sura_cameras",
                        "cameras.launch.py",
                        {
                            "environment": environment,
                            "robot_namespace": robot_namespace,
                        },
                    )
                ],
            ),
        ]
    )

    return launch_entities


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="bluerov"),
            DeclareLaunchArgument("robot_variant", default_value="auv"),
            DeclareLaunchArgument("arms", default_value=""),
            DeclareLaunchArgument("environment", default_value="sim"),
            DeclareLaunchArgument("localization", default_value="real"),
            DeclareLaunchArgument("use_imu_calibration", default_value="true"),
            DeclareLaunchArgument("alpha_use_fake_hardware", default_value="true"),
            DeclareLaunchArgument("alpha_left_serial_port", default_value=""),
            DeclareLaunchArgument("alpha_right_serial_port", default_value=""),
            DeclareLaunchArgument("alpha_left_state_update_frequency", default_value="250"),
            DeclareLaunchArgument("alpha_right_state_update_frequency", default_value="250"),
            DeclareLaunchArgument("initial_positions_file", default_value="initial_positions.yaml"),
            DeclareLaunchArgument("pressure_offset_pa", default_value="101325.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
