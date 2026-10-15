"""Check hardware ownership and controller endpoints against the actual robot profiles."""
import importlib.util
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "sura_controllers_launch", ROOT / "sura_bringup/launch/sura_controllers.launch.py"
)
CONTROLLERS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CONTROLLERS)


@pytest.mark.parametrize("arms,count", [("none", 0), ("single", 1), ("dual", 2)])
def test_hardware_ownership_and_profiles(arms, count):
    xacro = "cirtesub.urdf.xacro" if arms == "none" else "cirtesub_dual_alpha.urdf.xacro"
    original = ET.fromstring(subprocess.check_output([
        "xacro", str(ROOT / "cirtesub_description/urdf" / xacro), f"arms:={arms}"
    ], text=True))
    original_xml = ET.tostring(original)
    vehicle, alpha = CONTROLLERS.split_robot_description(original)
    vehicle_names = {node.get("name") for node in vehicle.findall("ros2_control")}
    alpha_names = {node.get("name") for node in alpha.findall("ros2_control")}
    assert len(alpha_names) == count
    assert not vehicle_names & alpha_names
    assert vehicle_names | alpha_names == {
        node.get("name") for node in original.findall("ros2_control")
    }
    assert ET.tostring(original) == original_xml
    for model in (vehicle, alpha):
        assert {node.get("name") for node in model.findall("link")} == {
            node.get("name") for node in original.findall("link")
        }
    suffix = "" if arms == "none" else f"_{arms}_alpha"
    filename = ROOT / f"cirtesub_description/config/ros2_control_params{suffix}.yaml"
    config = yaml.safe_load(filename.read_text())
    for namespace, model, mode in (("controller", vehicle, "none"), ("alpha/controller", alpha, arms)):
        if not model.findall("ros2_control"):
            continue
        parameters = config[f"/cirtesub/{namespace}/controller_manager"]["ros__parameters"]
        names = [name for name in CONTROLLERS.load_controllers_to_spawn(
            filename, "underwater", mode, model
        ) if isinstance(parameters.get(name), dict)]
        assert names
        if namespace == "controller":
            assert "body_velocity" in names
            assert not any(name.startswith("alpha_") for name in names)
            assert "task_priority_controller" not in names
        else:
            assert "alpha_left_forward_velocity_controller" in names
            assert ("alpha_right_forward_velocity_controller" in names) == (arms == "dual")
            assert ("task_priority_controller" in names) == (arms == "dual")


def test_mixed_hardware_is_rejected():
    root = ET.fromstring('''<robot><ros2_control name="mixed">
      <joint name="cirtesub/alpha_left/axis_b"/><joint name="cirtesub/thruster"/>
    </ros2_control></robot>''')
    with pytest.raises(RuntimeError, match="vehicle and Alpha joints"):
        CONTROLLERS.split_robot_description(root)


@pytest.mark.parametrize("profile", ["sim", "real"])
def test_teleop_endpoints_match_controller_profiles(profile):
    teleop = yaml.safe_load((ROOT / f"sura_teleop/config/teleop_params_cirtesub_{profile}.yaml").read_text())
    teleop = teleop["sura_teleop"]["ros__parameters"]
    config = yaml.safe_load((ROOT / "cirtesub_description/config/ros2_control_params_dual_alpha.yaml").read_text())
    manager = "/cirtesub/alpha/controller/controller_manager"
    parameters = config[manager]["ros__parameters"]
    assert teleop["alpha_controller_switch_service"].replace("/sura/", "/cirtesub/") == manager + "/switch_controller"
    for key, value in teleop.items():
        if key.startswith("alpha_") and isinstance(value, dict) and "name" in value:
            assert value["name"] in parameters
            node = "/cirtesub/alpha/controller/" + value["name"]
            assert node in config
            if "command_topic" in value:
                command = value["command_topic"].replace("/sura/", "/cirtesub/")
                assert command.startswith(node + "/")
                explicit = config[node]["ros__parameters"].get("command_topic")
                if explicit:
                    assert explicit == command


def test_task_action_remaps_cover_humble_transport_endpoints():
    remaps = dict(CONTROLLERS.task_priority_remappings(
        "cirtesub", {"task_ids": ["left_joint_trajectory"]}
    ))
    prefix = "/cirtesub/alpha/controller/task_priority/tasks/left_joint_trajectory"
    preserved = "/cirtesub/controller/task_priority/tasks/left_joint_trajectory"
    assert remaps[prefix + "/joint_trajectory"] == preserved + "/joint_trajectory"
    for suffix in ("send_goal", "get_result", "cancel_goal", "feedback", "status"):
        endpoint = "/follow_joint_trajectory/_action/" + suffix
        assert remaps[prefix + endpoint] == preserved + endpoint
    assert not any("*" in source for source in remaps)


def test_pose_feedforward_remaps_preserve_public_namespace():
    remaps = dict(CONTROLLERS.task_priority_remappings(
        "cirtesub", {"task_ids": ["left_pose", "right_pose", "base_pose", "stabilize", "left_arm_nominal", "right_arm_nominal"]}
    ))
    for task in ("left_pose", "right_pose", "base_pose", "stabilize"):
        assert remaps[f"/cirtesub/alpha/controller/task_priority/tasks/{task}/feedforward"] == (
            f"/cirtesub/controller/task_priority/tasks/{task}/feedforward"
        )
    for suffix in ("enable_roll_pitch", "disable_roll_pitch"):
        assert remaps[f"/cirtesub/alpha/controller/task_priority/tasks/stabilize/{suffix}"] == (
            f"/cirtesub/controller/task_priority/tasks/stabilize/{suffix}"
        )
    for task in ("left_arm_nominal", "right_arm_nominal"):
        assert remaps[f"/cirtesub/alpha/controller/task_priority/tasks/{task}/joint_command"] == (
            f"/cirtesub/controller/task_priority/tasks/{task}/joint_command"
        )
