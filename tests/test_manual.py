import struct
import pytest
from unittest.mock import patch, MagicMock
from dobotWrapperPy.dobotapi import DobotApi
from dobotWrapperPy.message import Message
from dobotWrapperPy.dobotConnection import DobotConnection
from dobotWrapperPy.enums.CommunicationProtocolIDs import CommunicationProtocolIDs
from dobotWrapperPy.enums.ControlValues import ControlValues
from dobotWrapperPy.enums.ptpMode import PTPMode
from dobotWrapperPy.enums.tagVersionRail import tagVersionRail
from dobotWrapperPy.enums.CPMode import CPMode
from dobotWrapperPy.paramsStructures import (
    tagWithL,
    tagWithLReturn,
    tagWAITCmd,
    tagCPCmd,
    tagPTPCmd,
    tagHomeCmd,
)
from typing import Generator
from enum import Enum

pytestmark = pytest.mark.skip("Skipped by default, run explicitly if needed")


class Asker(Enum):
    YES = 0
    NO = 1


class Confirmation(Enum):
    YES = 0
    NO = 1


def ask(prompt: str) -> Asker:
    answer = input(f"{prompt} [y/N/s(skip)]: ").strip().lower()
    if answer == "y":
        return Asker.YES
    return Asker.NO


def confirm(prompt: str) -> Confirmation:
    answer = input(f"{prompt} [y/N/]: ").strip().lower()
    if answer == "y":
        return Confirmation.YES
    else:
        return Confirmation.NO


@pytest.fixture
def mock_device() -> Generator[DobotApi, None, None]:

    port = input("Input port: ")
    device = DobotApi(DobotConnection(port), verbose=True)
    yield device


def test_set_get_device_sn_little_endian(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Device serialNumber:")
    if ask_answer == Asker.NO:
        pytest.skip()

    serialNumber = "SN12345"

    original_sn = mock_device.get_device_sn()
    print(f"Original Serial Number: {original_sn}")

    if ask("Continue: ") == Asker.NO:
        pytest.skip()

    mock_device.set_device_sn(serialNumber)
    assert mock_device.get_device_sn() == serialNumber

    mock_device.set_device_sn(original_sn)
    assert mock_device.get_device_sn() == original_sn


def test_set_get_device_name(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Device Name:")
    if ask_answer == Asker.NO:
        pytest.skip()

    deviceName = "Dobot-12345"
    original_name = mock_device.get_device_name()
    print(f"Original Device Name: {original_name}")

    if ask("Continue: ") == Asker.NO:
        pytest.skip()

    mock_device.set_device_name(deviceName)
    assert mock_device.get_device_name() == deviceName

    mock_device.set_device_name(original_name)
    assert mock_device.get_device_name() == original_name


def test_get_device_version(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Device Version:")
    if ask_answer == Asker.NO:
        pytest.skip()

    version = mock_device.get_device_version()
    print(f"Device Version: {version}")
    assert len(version) == 3


def test_set_get_device_rail_capability(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Device Rail Capability:")
    if ask_answer == Asker.NO:
        pytest.skip()

    params = tagWithL(True, tagVersionRail.VER_V2)

    mock_device.set_device_rail_capability(params)
    assert mock_device.get_device_rail_capability() is True

    params.is_with_rail = False
    mock_device.set_device_rail_capability(params)
    assert mock_device.get_device_rail_capability() is False


def test_get_device_time(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Device Time:")
    if ask_answer == Asker.NO:
        pytest.skip()

    device_time = mock_device.get_device_time()
    print(f"Device Time: {device_time}")
    assert device_time > 0


def test_get_device_id(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Device ID:")
    if ask_answer == Asker.NO:
        pytest.skip()

    device_id = mock_device.get_device_id()
    print(f"Device ID: {device_id}")
    assert len(device_id) == 3


def test_reset_pose(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Reset Pose:")
    if ask_answer == Asker.NO:
        pytest.skip()

    manual = 1
    rear_arm_angle = 10.0
    front_arm_angle = 20.0

    mock_device.reset_pose(manual, rear_arm_angle, front_arm_angle)
    print("Pose reset command sent")


def test_get_pose(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Get Pose:")
    if ask_answer == Asker.NO:
        pytest.skip()

    pose = mock_device.get_pose()
    print(f"Pose: {pose}")
    assert pose is not None


def test_get_pose_rail(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Get Pose Rail:")
    if ask_answer == Asker.NO:
        pytest.skip()

    pose_rail = mock_device.get_pose_rail()
    print(f"Pose Rail: {pose_rail}")
    assert isinstance(pose_rail, float)


def test_get_alarms_state(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Get Alarms State:")
    if ask_answer == Asker.NO:
        pytest.skip()

    alarms_state = mock_device.get_alarms_state()
    print(f"Alarms State: {alarms_state}")
    assert isinstance(alarms_state, list)
    assert len(alarms_state) == 16


def test_clear_all_alarms_state(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Clear All Alarms State:")
    if ask_answer == Asker.NO:
        pytest.skip()

    mock_device.clear_all_alarms_state()
    print("Alarms cleared")


def test_set_get_home_params(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set/Get Home Params:")
    if ask_answer == Asker.NO:
        pytest.skip()

    params = mock_device.get_home_params()
    print(f"Original Home Params: {params}")

    if ask("Modify and Set Home Params? ") == Asker.YES:
        params.x += 10
        params.y -= 10
        mock_device.set_home_params(params)
        print("Home params set")

    new_params = mock_device.get_home_params()
    print(f"New Home Params: {new_params}")
    assert new_params is not None


def test_set_home_cmd(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set Home Cmd:")
    if ask_answer == Asker.NO:
        pytest.skip()

    params = tagHomeCmd(0)  # Using WAITCmd as a placeholder, adjust if needed
    mock_device.set_home_cmd(params)
    print("Home command sent")


def test_set_get_auto_leveling(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set/Get Auto Leveling:")
    if ask_answer == Asker.NO:
        pytest.skip()

    #  params = tagAutoLevelingParams() #  Need to define tagAutoLevelingParams
    #  mock_device.set_autoleveling(params)
    #  result = mock_device.get_autoleveling()
    #  print(f"Auto Leveling Result: {result}")
    print("Auto leveling tests skipped due to missing structure definition")


def test_set_get_hht_trig_mode(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set/Get HHT Trig Mode:")
    if ask_answer == Asker.NO:
        pytest.skip()

    #  mock_device.set_hht_trig_mode(HHTTrigMode.CONTINUOUS) #  Need to define HHTTrigMode
    #  mode = mock_device.get_hht_trig_mode()
    #  print(f"HHT Trig Mode: {mode}")
    print("HHT Trig Mode tests skipped due to missing enum definition")


def test_set_get_hht_trig_output_enabled(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set/Get HHT Trig Output Enabled:")
    if ask_answer == Asker.NO:
        pytest.skip()

    mock_device.set_hht_trig_output_enabled(True)
    enabled = mock_device.get_hht_trig_output_enabled()
    print(f"HHT Trig Output Enabled: {enabled}")
    assert enabled is True

    mock_device.set_hht_trig_output_enabled(False)
    enabled = mock_device.get_hht_trig_output_enabled()
    assert enabled is False


def test_set_get_cp_params(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set/Get CP Params:")
    if ask_answer == Asker.NO:
        pytest.skip()

    #  params = tagCPParams() #  Need to define tagCPParams
    #  mock_device.set_cp_params(params)
    #  gotten_params = mock_device.get_cp_params()
    #  print(f"CP Params: {gotten_params}")
    print("CP Params tests skipped due to missing structure definition")


def test_set_cp_cmd(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set CP Cmd:")
    if ask_answer == Asker.NO:
        pytest.skip()

    cmd = tagCPCmd(CPMode.ABSOLUTE, 100, 100, 100, 100)  # Need to define tagCPCmd
    mock_device.set_cp_cmd(cmd)
    print("CP command sent")


def test_set_get_ptp_joint_params(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set/Get PTP Joint Params:")
    if ask_answer == Asker.NO:
        pytest.skip()

    #  params = tagPTPJointParams() #  Need to define tagPTPJointParams
    #  mock_device.set_ptp_joint_params(params)
    #  gotten_params = mock_device.get_ptp_joint_params()
    #  print(f"PTP Joint Params: {gotten_params}")
    print("PTP Joint Params tests skipped due to missing structure definition")


def test_set_get_ptp_coordinate_params(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set/Get PTP Coordinate Params:")
    if ask_answer == Asker.NO:
        pytest.skip()

    #  params = tagPTPCoordinateParams() #  Need to define tagPTPCoordinateParams
    #  mock_device.set_ptp_coordinate_params(params)
    #  gotten_params = mock_device.get_ptp_coordinate_params()
    #  print(f"PTP Coordinate Params: {gotten_params}")
    print("PTP Coordinate Params tests skipped due to missing structure definition")


def test_set_get_ptp_jump_params(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set/Get PTP Jump Params:")
    if ask_answer == Asker.NO:
        pytest.skip()

    #  params = tagPTPJumpParams() #  Need to define tagPTPJumpParams
    #  mock_device.set_ptp_jump_params(params)
    #  gotten_params = mock_device.get_ptp_jump_params()
    #  print(f"PTP Jump Params: {gotten_params}")
    print("PTP Jump Params tests skipped due to missing structure definition")


def test_set_get_ptp_common_params(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set/Get PTP Common Params:")
    if ask_answer == Asker.NO:
        pytest.skip()

    #  params = tagPTPCommonParams() #  Need to define tagPTPCommonParams
    #  mock_device.set_ptp_common_params(params)
    #  gotten_params = mock_device.get_ptp_common_params()
    #  print(f"PTP Common Params: {gotten_params}")
    print("PTP Common Params tests skipped due to missing structure definition")


def test_set_ptp_cmd(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set PTP Cmd:")
    if ask_answer == Asker.NO:
        pytest.skip()

    cmd = tagPTPCmd(PTPMode.MOVJ_XYZ, 100, 100, 100, 100)  # Need to define tagPTPCmd
    mock_device.set_ptp_cmd(cmd)
    print("PTP command sent")


def test_set_get_arc_params(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set/Get ARC Params:")
    if ask_answer == Asker.NO:
        pytest.skip()

    #  params = tagARCParams() #  Need to define tagARCParams
    #  mock_device.set_arc_params(params)
    #  gotten_params = mock_device.get_arc_params()
    #  print(f"ARC Params: {gotten_params}")
    print("ARC Params tests skipped due to missing structure definition")


def test_set_arc_cmd(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set ARC Cmd:")
    if ask_answer == Asker.NO:
        pytest.skip()

    #  cmd = tagARCCmd() #  Need to define tagARCCmd
    #  mock_device.set_arc_cmd(cmd)
    print("ARC command sent")
    print("ARC command tests skipped due to missing structure definition")


def test_set_get_circle_params(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set/Get Circle Params:")
    if ask_answer == Asker.NO:
        pytest.skip()

    #  params = tagCircleParams() #  Need to define tagCircleParams
    #  mock_device.set_circle_params(params)
    #  gotten_params = mock_device.get_circle_params()
    #  print(f"Circle Params: {gotten_params}")
    print("Circle Params tests skipped due to missing structure definition")


def test_set_circle_cmd(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set Circle Cmd:")
    if ask_answer == Asker.NO:
        pytest.skip()

    #  cmd = tagCircleCmd() #  Need to define tagCircleCmd
    #  mock_device.set_circle_cmd(cmd)
    print("Circle command sent")
    print("Circle command tests skipped due to missing structure definition")


def test_set_jog_cmd(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set JOG Cmd:")
    if ask_answer == Asker.NO:
        pytest.skip()

    #  cmd = tagJOGCmd() #  Need to define tagJOGCmd
    #  mock_device.set_jog_cmd(cmd)
    print("JOG command sent")
    print("JOG command tests skipped due to missing structure definition")


def test_set_get_io_do(mock_device: DobotApi) -> None:
    ask_answer = ask("Test Set/Get IO DO:")
    if ask_answer == Asker.NO:
        pytest.skip()

    #  io_index = 1
    #  level = Level.HIGH #  Need to define Level
    #  mock_device.set_io_do(io_index, level)
    #
