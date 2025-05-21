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
)
from typing import Generator


@pytest.fixture
def mock_conn() -> Generator[DobotConnection, None, None]:
    mock_serial_instance = MagicMock()
    mock_serial_instance.isOpen.return_value = True
    mock_serial_instance.name = "COM1"
    mock_serial_instance.read_all.return_value = b""
    conn = DobotConnection(serial_conn=mock_serial_instance)
    yield conn


@pytest.fixture
def mock_device(mock_conn: DobotConnection) -> Generator[DobotApi, None, None]:

    with patch.object(DobotApi, "set_queued_cmd_start_exec"), patch.object(
        DobotApi, "set_queued_cmd_clear"
    ), patch.object(DobotApi, "set_ptp_joint_params"), patch.object(
        DobotApi, "set_ptp_coordinate_params"
    ), patch.object(
        DobotApi, "set_ptp_jump_params"
    ), patch.object(
        DobotApi, "set_ptp_common_params"
    ), patch.object(
        DobotApi, "get_pose"
    ):

        device = DobotApi(mock_conn, verbose=True)
        yield device


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_set_device_sn_little_endian(
    mock_send_command: MagicMock, mock_device: DobotApi
) -> None:
    serialNumber = "SN12345"

    mock_device.set_device_sn(serialNumber)

    mock_send_command.assert_called_once()

    print(mock_send_command.call_args)
    (packet_message, _), _ = mock_send_command.call_args
    print(packet_message)
    assert isinstance(packet_message, Message)

    packet: bytes = packet_message.bytes()

    assert packet[:2] == b"\xaa\xaa"

    expected_payload_length: int = 2 + len(serialNumber.encode("utf-8"))
    assert packet[2] == expected_payload_length

    payload: bytes = packet[3:-1]

    assert payload[0] == 0x00  # ID
    assert payload[1] == 0x01  # Ctrl: rw=1, isQueued=0
    assert payload[2:] == serialNumber.encode("utf-8")

    assert packet


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_get_device_sn_little_endian(
    mock_send_command: MagicMock, mock_device: DobotApi
) -> None:
    serialNumber = "SN12345"
    returnMsg = Message()
    returnMsg.id = CommunicationProtocolIDs.GET_SET_DEVICE_SN
    returnMsg.ctrl = ControlValues.ReadWrite
    returnMsg.params = bytearray([])
    returnMsg.params.extend(serialNumber.encode("utf-8"))

    mock_send_command.return_value = returnMsg

    return_packet = mock_device.get_device_sn()
    assert return_packet == returnMsg

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    packet: bytes = packet_message.bytes()

    assert packet[:2] == b"\xaa\xaa"

    expected_payload_length: int = 2
    assert packet[2] == expected_payload_length

    payload: bytes = packet[3:-1]

    assert payload[0] == 0x00  # ID
    assert payload[1] == ControlValues.Zero.value  # Ctrl: rw=1, isQueued=0
    assert payload[2:] == b""

    assert packet


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_get_queued_cmd_current_index_message(
    mock_send_command: MagicMock, mock_device: DobotApi
) -> None:
    """Test the message sent for getting the current command index and its response."""
    # Mock the response with a dummy index
    mock_response_msg = MagicMock(spec=Message)
    dummy_index = 42
    mock_response_msg.params = struct.pack(
        "<L", dummy_index
    )  # Little-endian unsigned long
    mock_send_command.return_value = mock_response_msg

    returned_index = mock_device.get_queued_cmd_current_index()

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.GET_QUEUED_CMD_CURRENT_INDEX
    assert packet_message.ctrl == ControlValues.Zero  # GET command should have Ctrl=0
    assert packet_message.params == bytearray(
        []
    )  # GET command should have empty params

    # Verify the returned value from the method matches the mocked response
    assert returned_index == dummy_index


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_set_cp_cmd_little_endian(
    mock_send_command: MagicMock, mock_device: DobotApi
) -> None:
    """Test setting CP command with little-endian float parameters."""
    x, y, z = 100.0, 50.0, 25.0
    cpCmd = tagCPCmd(CPMode.ABSOLUTE, x, y, z, 100)

    # Mock the response for the SET command (usually contains the command index)
    mock_response_msg = MagicMock(spec=Message)
    mock_response_msg.params = struct.pack("<L", 124)  # Mock a command index response
    mock_send_command.return_value = mock_response_msg

    mock_device.set_cp_cmd(cpCmd)

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.SET_CP_CMD
    assert packet_message.ctrl == ControlValues.Both

    # Verify payload structure and endianness
    expected_params = bytearray(cpCmd.pack())  # CP mode
    # expected_params.append(0x00)  # Reserved byte

    assert packet_message.params == expected_params


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_set_end_effector_gripper_little_endian(
    mock_send_command: MagicMock, mock_device: DobotApi
) -> None:
    """Test setting gripper status with little-endian boolean parameter."""
    enable = True

    # Mock the response for the SET command (usually contains the command index)
    mock_response_msg = MagicMock(spec=Message)
    mock_response_msg.params = struct.pack("<L", 125)  # Mock a command index response
    mock_send_command.return_value = mock_response_msg

    mock_device.set_end_effector_gripper(enable=enable)

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.SET_GET_END_EFFECTOR_GRIPPER
    assert packet_message.ctrl == ControlValues.Both

    # Verify payload structure and endianness
    expected_params = bytearray([0x01, 0x01 if enable else 0x00])

    assert packet_message.params == expected_params

    # Test with disable
    mock_send_command.reset_mock()
    # Mock the response for the SET command (usually contains the command index)
    mock_response_msg_disable = MagicMock(spec=Message)
    mock_response_msg_disable.params = struct.pack(
        "<L", 126
    )  # Mock a command index response
    mock_send_command.return_value = mock_response_msg_disable

    mock_device.set_end_effector_gripper(enable=False)
    mock_send_command.assert_called_once()
    (packet_message_disable, _), _ = mock_send_command.call_args
    assert packet_message_disable.params == bytearray([0x01, 0x00])


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_set_end_effector_suction_cup_little_endian(
    mock_send_command: MagicMock, mock_device: DobotApi
) -> None:
    """Test setting suction cup status with little-endian boolean parameter."""
    enable = True

    # Mock the response for the SET command (usually contains the command index)
    mock_response_msg = MagicMock(spec=Message)
    mock_response_msg.params = struct.pack("<L", 127)  # Mock a command index response
    mock_send_command.return_value = mock_response_msg

    mock_device.set_end_effector_suction_cup(enable=enable)

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert (
        packet_message.id == CommunicationProtocolIDs.SET_GET_END_EFFECTOR_SUCTION_CUP
    )
    assert packet_message.ctrl == ControlValues.Both

    # Verify payload structure and endianness
    expected_params = bytearray([0x01, 0x01 if enable else 0x00])

    assert packet_message.params == expected_params

    # Test with disable
    mock_send_command.reset_mock()
    # Mock the response for the SET command (usually contains the command index)
    mock_response_msg_disable = MagicMock(spec=Message)
    mock_response_msg_disable.params = struct.pack(
        "<L", 128
    )  # Mock a command index response
    mock_send_command.return_value = mock_response_msg_disable

    mock_device.set_end_effector_suction_cup(enable=False)
    mock_send_command.assert_called_once()
    (packet_message_disable, _), _ = mock_send_command.call_args
    assert packet_message_disable.params == bytearray([0x01, 0x00])


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_set_ptp_cmd_little_endian(
    mock_send_command: MagicMock, mock_device: DobotApi
) -> None:
    """Test setting PTP command with little-endian parameters."""
    x, y, z, r = 250.0, 0.0, 50.0, 0.0
    mode = PTPMode.MOVJ_XYZ
    wait = False  # wait parameter does not affect the message content

    # Mock the response for the SET command (usually contains the command index)
    mock_response_msg = MagicMock(spec=Message)
    mock_response_msg.params = struct.pack("<L", 133)  # Mock a command index response
    mock_send_command.return_value = mock_response_msg

    mock_device.set_ptp_cmd(tagPTPCmd(PTPMode.MOVJ_XYZ, x, y, z, r), wait)

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.SET_PTP_CMD
    assert packet_message.ctrl == ControlValues.Both

    # Verify payload structure and endianness
    expected_params = bytearray([mode.value])
    expected_params.extend(struct.pack("<f", x))
    expected_params.extend(struct.pack("<f", y))
    expected_params.extend(struct.pack("<f", z))
    expected_params.extend(struct.pack("<f", r))

    assert packet_message.params == expected_params


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_set_wait_cmd_little_endian(
    mock_send_command: MagicMock, mock_device: DobotApi
) -> None:
    """Test setting wait command with little-endian integer parameter and its response."""
    ms = 1000  # milliseconds
    tag = tagWAITCmd(ms)

    # Mock the response for the SET command (usually contains the command index)
    mock_response_msg = MagicMock(spec=Message)
    mock_response_msg.params = tag.pack()  # Mock a command index response
    mock_send_command.return_value = mock_response_msg

    mock_device.set_wait_cmd(tag)

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.SET_WAIT_CMD
    assert packet_message.ctrl == ControlValues.Both

    # Verify payload structure and endianness
    expected_params = bytearray(struct.pack("<I", ms))

    assert packet_message.params == expected_params


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_set_queued_cmd_stop_exec_message(
    mock_send_command: MagicMock, mock_device: DobotApi
) -> None:
    """Test the message sent for stopping queued command execution and its response."""
    # Mock the response for the SET command (usually contains the command index)
    mock_response_msg = MagicMock(spec=Message)
    mock_response_msg.params = struct.pack("<L", 137)  # Mock a command index response
    mock_send_command.return_value = mock_response_msg

    mock_device.set_queued_cmd_stop_exec()

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.SET_QUEUED_CMD_STOP_EXEC
    assert packet_message.ctrl == ControlValues.ReadWrite
    assert packet_message.params == bytearray([])  # SET command with no parameters


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_get_device_sn_message(
    mock_send_command: MagicMock, mock_device: DobotApi
) -> None:
    """Test the message sent for getting the device serial number and its response."""
    # Mock the response with a dummy serial number string
    mock_response_msg = MagicMock(spec=Message)
    dummy_serial_number = "MOCKED_SN123"
    mock_response_msg.params = dummy_serial_number.encode("utf-8")
    mock_send_command.return_value = mock_response_msg

    returned_message = mock_device.get_device_sn()

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.GET_SET_DEVICE_SN
    assert packet_message.ctrl == ControlValues.Zero  # GET command should have Ctrl=0
    assert packet_message.params == bytearray(
        []
    )  # GET command should have empty params

    # Verify the returned message is the mocked response
    assert returned_message == mock_response_msg


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_set_device_name_little_endian(
    mock_send_command: MagicMock, mock_device: DobotApi
) -> None:
    """Test setting device name with little-endian encoding."""
    device_name = "MyDobot"

    # Mock the response for the SET command (usually contains the command index)
    mock_response_msg = MagicMock(spec=Message)
    mock_response_msg.params = struct.pack("<L", 138)  # Mock a command index response
    mock_send_command.return_value = mock_response_msg

    mock_device.set_device_name(device_name)

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.GET_SET_DEVICE_NAME
    assert packet_message.ctrl == ControlValues.ReadWrite

    # Verify payload structure and endianness
    expected_params = bytearray(device_name.encode("utf-8"))

    assert packet_message.params == expected_params


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_get_device_name_message(
    mock_send_command: MagicMock, mock_device: DobotApi
) -> None:
    """Test the message sent for getting the device name and its response."""
    # Mock the response with a dummy device name string
    mock_response_msg = MagicMock(spec=Message)
    dummy_device_name = "MOCKED_DOBOT"
    mock_response_msg.params = dummy_device_name.encode("utf-8")
    mock_send_command.return_value = mock_response_msg

    returned_message = mock_device.get_device_name()

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.GET_SET_DEVICE_NAME
    assert packet_message.ctrl == ControlValues.Zero  # GET command should have Ctrl=0
    assert packet_message.params == bytearray(
        []
    )  # GET command should have empty params

    # Verify the returned message is the mocked response
    assert returned_message == mock_response_msg


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_get_device_version_message(
    mock_send_command: MagicMock, mock_device: DobotApi
) -> None:
    """Test the message sent for getting the device version and its response."""
    # Mock the response with dummy version bytes (major, minor, revision)
    mock_response_msg = MagicMock(spec=Message)
    dummy_version = (1, 2, 3)
    mock_response_msg.params = bytes(dummy_version)
    mock_send_command.return_value = mock_response_msg

    returned_message = mock_device.get_device_version()

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.GET_DEVICE_VERSION
    assert packet_message.ctrl == ControlValues.Zero  # GET command should have Ctrl=0
    assert packet_message.params == bytearray(
        []
    )  # GET command should have empty params

    # Verify the returned message is the mocked response
    assert returned_message == mock_response_msg


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_set_device_rail_capability_little_endian(
    mock_send_command: MagicMock, mock_device: DobotApi
) -> None:
    """Test setting device rail capability with little-endian encoding."""
    enable = True
    version = tagVersionRail.VER_V2

    # Mock the response for the SET command (usually contains the command index)
    mock_response_msg = MagicMock(spec=Message)
    mock_response_msg.params = struct.pack("<L", 139)  # Mock a command index response
    mock_send_command.return_value = mock_response_msg

    mock_device.set_device_rail_capability(enable, version)

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.SET_GET_DEVICE_WITH_RAIL
    assert packet_message.ctrl == ControlValues.ReadWrite

    # Verify payload structure and endianness using the tagWithL structure
    expected_tag = tagWithL(enable, version)
    expected_params = bytearray(expected_tag.pack())

    assert packet_message.params == expected_params


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_get_device_rail_capability_little_endian(
    mock_send_command: MagicMock, mock_device: DobotApi
) -> None:
    """Test the message sent for getting the device version and its response."""
    # Mock the response with dummy version bytes (major, minor, revision)
    mock_response_msg = MagicMock(spec=Message)
    mock_response_msg.params = tagWithLReturn(True)
    mock_send_command.return_value = mock_response_msg

    returned_message = mock_device.get_device_rail_capability()
    assert returned_message == mock_response_msg

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.SET_GET_DEVICE_WITH_RAIL
    assert packet_message.ctrl == ControlValues.Zero  # GET command should have Ctrl=0
    assert packet_message.params == bytearray(
        []
    )  # GET command should have empty params

    # Verify the returned message is the mocked response
    assert returned_message == mock_response_msg
