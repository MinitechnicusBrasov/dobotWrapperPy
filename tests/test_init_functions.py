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
from dobotWrapperPy.paramsStructures import (
    tagPose,
    tagPTPJointParams,
    tagPTPCoordinateParams,
    tagPTPJumpParams,
    tagPTPCommonParams,
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
def mock_device_normal(mock_conn: DobotConnection) -> Generator[DobotApi, None, None]:

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


@pytest.fixture
def mock_device_without_set_queued_cmd_start_exec(
    mock_conn: DobotConnection,
) -> Generator[DobotApi, None, None]:

    with patch.object(DobotApi, "set_queued_cmd_clear"), patch.object(
        DobotApi, "set_ptp_joint_params"
    ), patch.object(DobotApi, "set_ptp_coordinate_params"), patch.object(
        DobotApi, "set_ptp_jump_params"
    ), patch.object(
        DobotApi, "set_ptp_common_params"
    ), patch.object(
        DobotApi, "get_pose"
    ), patch.object(
        DobotApi, "_send_command"
    ):

        device = DobotApi(mock_conn, verbose=True)
        yield device


@pytest.fixture
def mock_device_without_set_ptp_joint_params(
    mock_conn: DobotConnection,
) -> Generator[DobotApi, None, None]:

    with patch.object(DobotApi, "set_queued_cmd_start_exec"), patch.object(
        DobotApi, "set_queued_cmd_clear"
    ), patch.object(DobotApi, "set_ptp_coordinate_params"), patch.object(
        DobotApi, "set_ptp_jump_params"
    ), patch.object(
        DobotApi, "set_ptp_common_params"
    ), patch.object(
        DobotApi, "get_pose"
    ), patch.object(
        DobotApi, "_send_command"
    ):

        device = DobotApi(mock_conn, verbose=True)
        yield device


@pytest.fixture
def mock_device_without_set_queued_cmd_clear(
    mock_conn: DobotConnection,
) -> Generator[DobotApi, None, None]:

    with patch.object(DobotApi, "set_queued_cmd_start_exec"), patch.object(
        DobotApi, "set_ptp_joint_params"
    ), patch.object(DobotApi, "set_ptp_coordinate_params"), patch.object(
        DobotApi, "set_ptp_jump_params"
    ), patch.object(
        DobotApi, "set_ptp_common_params"
    ), patch.object(
        DobotApi, "get_pose"
    ), patch.object(
        DobotApi, "_send_command"
    ):

        device = DobotApi(mock_conn, verbose=True)
        yield device


@pytest.fixture
def mock_device_without_set_ptp_coordinate_params(
    mock_conn: DobotConnection,
) -> Generator[DobotApi, None, None]:

    with patch.object(DobotApi, "set_queued_cmd_start_exec"), patch.object(
        DobotApi, "set_queued_cmd_clear"
    ), patch.object(DobotApi, "set_ptp_joint_params"), patch.object(
        DobotApi, "set_ptp_jump_params"
    ), patch.object(
        DobotApi, "set_ptp_common_params"
    ), patch.object(
        DobotApi, "get_pose"
    ), patch.object(
        DobotApi, "_send_command"
    ):

        device = DobotApi(mock_conn, verbose=True)
        yield device


@pytest.fixture
def mock_device_without_set_ptp_jump_params(
    mock_conn: DobotConnection,
) -> Generator[DobotApi, None, None]:

    with patch.object(DobotApi, "set_queued_cmd_start_exec"), patch.object(
        DobotApi, "set_queued_cmd_clear"
    ), patch.object(DobotApi, "set_ptp_joint_params"), patch.object(
        DobotApi, "set_ptp_coordinate_params"
    ), patch.object(
        DobotApi, "set_ptp_common_params"
    ), patch.object(
        DobotApi, "get_pose"
    ), patch.object(
        DobotApi, "_send_command"
    ):

        device = DobotApi(mock_conn, verbose=True)
        yield device


@pytest.fixture
def mock_device_without_set_ptp_common_params(
    mock_conn: DobotConnection,
) -> Generator[DobotApi, None, None]:

    with patch.object(DobotApi, "set_queued_cmd_start_exec"), patch.object(
        DobotApi, "set_queued_cmd_clear"
    ), patch.object(DobotApi, "set_ptp_joint_params"), patch.object(
        DobotApi, "set_ptp_coordinate_params"
    ), patch.object(
        DobotApi, "set_ptp_jump_params"
    ), patch.object(
        DobotApi, "get_pose"
    ), patch.object(
        DobotApi, "_send_command"
    ):

        device = DobotApi(mock_conn, verbose=True)
        yield device


@pytest.fixture
def mock_device_without_get_pose(
    mock_conn: DobotConnection,
) -> Generator[DobotApi, None, None]:

    with patch.object(DobotApi, "set_queued_cmd_start_exec"), patch.object(
        DobotApi, "set_queued_cmd_clear"
    ), patch.object(DobotApi, "set_ptp_joint_params"), patch.object(
        DobotApi, "set_ptp_coordinate_params"
    ), patch.object(
        DobotApi, "set_ptp_jump_params"
    ), patch.object(
        DobotApi, "set_ptp_common_params"
    ), patch.object(
        DobotApi, "_send_command"
    ) as mock_send_command:
        mock_response_msg = MagicMock(spec=Message)
        dummy_pose_data = (100.0, 50.0, 25.0, 0.0, 10.0, 20.0, 30.0, 10.0)
        mock_response_msg.params = struct.pack("<8f", *dummy_pose_data)
        mock_send_command.return_value = mock_response_msg

        device = DobotApi(mock_conn, verbose=True)
        yield device


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_get_pose_message(
    mock_send_command: MagicMock, mock_device_without_get_pose: DobotApi
) -> None:
    """Test the message sent for getting the current pose and its response."""
    # Mock the response with dummy pose data (8 floats)
    mock_response_msg = MagicMock(spec=Message)
    dummy_pose_data = (100.0, 50.0, 25.0, 0.0, 10.0, 20.0, 30.0, 40.0)
    mock_response_msg.params = tagPose(
        dummy_pose_data[0],
        dummy_pose_data[1],
        dummy_pose_data[2],
        dummy_pose_data[3],
        list(dummy_pose_data[4:]),
    ).pack()
    mock_send_command.return_value = mock_response_msg

    returned_message = mock_device_without_get_pose.get_pose()
    assert isinstance(returned_message, tagPose)

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.GET_POSE
    assert packet_message.ctrl == ControlValues.Zero  # GET command should have Ctrl=0
    assert packet_message.params == bytearray(
        []
    )  # GET command should have empty params

    # Verify the returned message is the mocked response
    assert returned_message == tagPose.unpack(mock_response_msg.params)

    # Verify the internal pose attributes were updated


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_set_ptp_joint_params_little_endian(
    mock_send_command: MagicMock, mock_device_without_set_ptp_joint_params: DobotApi
) -> None:
    """Test setting PTP joint parameters with little-endian float parameters."""
    v_x, v_y, v_z, v_r = 200.0, 200.0, 200.0, 200.0
    a_x, a_y, a_z, a_r = 200.0, 200.0, 200.0, 200.0

    # Mock the response for the SET command (usually contains the command index)
    mock_response_msg = MagicMock(spec=Message)
    mock_response_msg.params = struct.pack("<L", 129)  # Mock a command index response
    mock_send_command.return_value = mock_response_msg

    mock_device_without_set_ptp_joint_params.set_ptp_joint_params(
        tagPTPJointParams([v_x, v_y, v_z, v_r], [a_x, a_y, a_z, a_r])
    )

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.SET_GET_PTP_JOINT_PARAMS
    assert packet_message.ctrl == ControlValues.ReadWrite

    # Verify payload structure and endianness
    expected_params = bytearray([])
    expected_params.extend(struct.pack("<f", v_x))
    expected_params.extend(struct.pack("<f", v_y))
    expected_params.extend(struct.pack("<f", v_z))
    expected_params.extend(struct.pack("<f", v_r))
    expected_params.extend(struct.pack("<f", a_x))
    expected_params.extend(struct.pack("<f", a_y))
    expected_params.extend(struct.pack("<f", a_z))
    expected_params.extend(struct.pack("<f", a_r))

    assert packet_message.params == expected_params


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_set_ptp_coordinate_params_little_endian(
    mock_send_command: MagicMock,
    mock_device_without_set_ptp_coordinate_params: DobotApi,
) -> None:
    """Test setting PTP coordinate parameters with little-endian float parameters."""
    velocity, acceleration = 200.0, 200.0

    # Mock the response for the SET command (usually contains the command index)
    mock_response_msg = MagicMock(spec=Message)
    mock_response_msg.params = struct.pack("<L", 130)  # Mock a command index response
    mock_send_command.return_value = mock_response_msg

    mock_device_without_set_ptp_coordinate_params.set_ptp_coordinate_params(
        tagPTPCoordinateParams(velocity, velocity, acceleration, acceleration)
    )

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.SET_GET_PTP_COORDINATE_PARAMS
    assert packet_message.ctrl == ControlValues.ReadWrite

    # Verify payload structure and endianness
    expected_params = bytearray([])
    expected_params.extend(struct.pack("<f", velocity))
    expected_params.extend(struct.pack("<f", velocity))  # velocity repeated
    expected_params.extend(struct.pack("<f", acceleration))
    expected_params.extend(struct.pack("<f", acceleration))  # acceleration repeated

    assert packet_message.params == expected_params


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_set_ptp_jump_params_little_endian(
    mock_send_command: MagicMock, mock_device_without_set_ptp_jump_params: DobotApi
) -> None:
    """Test setting PTP jump parameters with little-endian float parameters."""
    jump, limit = 10.0, 200.0

    # Mock the response for the SET command (usually contains the command index)
    mock_response_msg = MagicMock(spec=Message)
    mock_response_msg.params = struct.pack("<L", 131)  # Mock a command index response
    mock_send_command.return_value = mock_response_msg

    mock_device_without_set_ptp_jump_params.set_ptp_jump_params(
        tagPTPJumpParams(jump, limit)
    )

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.SET_GET_PTP_JUMP_PARAMS
    assert packet_message.ctrl == ControlValues.ReadWrite

    # Verify payload structure and endianness
    expected_params = bytearray([])
    expected_params.extend(struct.pack("<f", jump))
    expected_params.extend(struct.pack("<f", limit))

    assert packet_message.params == expected_params


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_set_ptp_common_params_little_endian(
    mock_send_command: MagicMock, mock_device_without_set_ptp_common_params: DobotApi
) -> None:
    """Test setting PTP common parameters with little-endian float parameters."""
    velocity, acceleration = 100.0, 100.0

    # Mock the response for the SET command (usually contains the command index)
    mock_response_msg = MagicMock(spec=Message)
    mock_response_msg.params = struct.pack("<L", 132)  # Mock a command index response
    mock_send_command.return_value = mock_response_msg

    mock_device_without_set_ptp_common_params.set_ptp_common_params(
        tagPTPCommonParams(velocity, acceleration)
    )

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.SET_GET_PTP_COMMON_PARAMS
    assert packet_message.ctrl == ControlValues.ReadWrite

    # Verify payload structure and endianness
    expected_params = bytearray([])
    expected_params.extend(struct.pack("<f", velocity))
    expected_params.extend(struct.pack("<f", acceleration))

    assert packet_message.params == expected_params


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_set_queued_cmd_clear_message(
    mock_send_command: MagicMock, mock_device_without_set_queued_cmd_clear: DobotApi
) -> None:
    """Test the message sent for clearing the command queue and its response."""
    # Mock the response for the SET command (usually contains the command index)
    mock_response_msg = MagicMock(spec=Message)
    mock_response_msg.params = struct.pack("<L", 134)  # Mock a command index response
    mock_send_command.return_value = mock_response_msg

    mock_device_without_set_queued_cmd_clear.set_queued_cmd_clear()

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.SET_QUEUED_CMD_CLEAR
    assert packet_message.ctrl == ControlValues.ReadWrite
    assert packet_message.params == bytearray([])  # SET command with no parameters


@patch("dobotWrapperPy.dobotapi.DobotApi._send_command")
def test_set_queued_cmd_start_exec_message(
    mock_send_command: MagicMock,
    mock_device_without_set_queued_cmd_start_exec: DobotApi,
) -> None:
    """Test the message sent for starting queued command execution and its response."""
    # Mock the response for the SET command (usually contains the command index)
    mock_response_msg = MagicMock(spec=Message)
    mock_response_msg.params = struct.pack("<L", 135)  # Mock a command index response
    mock_send_command.return_value = mock_response_msg

    mock_device_without_set_queued_cmd_start_exec.set_queued_cmd_start_exec()

    mock_send_command.assert_called_once()

    (packet_message, _), _ = mock_send_command.call_args
    assert isinstance(packet_message, Message)

    assert packet_message.id == CommunicationProtocolIDs.SET_QUEUED_CMD_START_EXEC
    assert packet_message.ctrl == ControlValues.ReadWrite
    assert packet_message.params == bytearray([])  # SET command with no parameters
