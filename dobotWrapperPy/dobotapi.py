# mypy: disable-error-code="import"
import struct
import threading
import time
import warnings
from typing import Optional, List, Tuple
from .enums.level import Level
from .dobotConnection import DobotConnection
from .enums.CommunicationProtocolIDs import CommunicationProtocolIDs
from .enums.ControlValues import ControlValues
from .enums.HHTTrigMode import HHTTrigMode
from .enums.ptpMode import PTPMode

# Unused import: tagVersionRail - kept as per original
# from .enums.tagVersionRail import tagVersionRail
from .message import Message
from .paramsStructures import (
    tagAutoLevelingParams,
    tagHomeCmd,
    tagHomeParams,
    tagARCCmd,
    tagARCParams,
    tagCPCmd,
    tagCPParams,
    tagDevice,
    tagEMOTOR,
    tagEndEffectorParams,
    tagIODO,
    tagIOMultiplexing,
    tagIOPWM,
    tagJOGCmd,
    tagJOGCommonParams,
    tagJOGCoordinateParams,
    tagJOGJointParams,
    tagJOGLParams,
    tagPOCmd,
    tagPose,
    tagPTPCmd,
    tagPTPCommonParams,
    tagPTPCoordinateParams,
    tagPTPJointParams,
    tagPTPJump2Params,
    tagPTPJumpParams,
    tagPTPLParams,
    tagPTPWithLCmd,
    tagTRIGCmd,
    tagWAITCmd,
    tagWIFIDNS,
    tagWIFIGateway,
    tagWIFIIPAddress,
    tagWIFINetmask,
    tagWithL,
)


class DobotApi(threading.Thread):
    """
    Initializes the Dobot API for communication and control of a Dobot robot
    arm. It manages the serial connection, command locking, and provides
    methods for various robot operations.
    """

    _on: bool
    verbose: bool
    lock: threading.Lock
    conn: DobotConnection
    # is_open: bool # This was set but not used elsewhere in the class
    # ctrl: ControlValues # This field was defined but not used

    def __init__(
        self, dobot_connection: DobotConnection, verbose: bool = False
    ) -> None:
        """
        Constructor for the DobotApi class.

        Args:
            dobot_connection (DobotConnection): An instance of DobotConnection
                handling the serial communication.
            verbose: If True, enables verbose output for debugging
                (default: False).
        """
        threading.Thread.__init__(self)

        self._on = True
        self.verbose = verbose
        self.lock = threading.Lock()
        self.conn = dobot_connection
        is_open = self.conn.serial_conn.isOpen()
        if self.verbose:
            print(
                "pydobot: %s open" % self.conn.serial_conn.name
                if is_open
                else "pydobot: failed to open serial port"
            )

        self._initialize_robot()

    def close(self) -> None:
        """
        Closes the serial connection to the Dobot and releases the lock.
        """
        self._on = False
        self.lock.acquire()
        try:
            if hasattr(self, "conn") and self.conn is not None:
                # DobotConnection destructor should handle serial_conn.close()
                del self.conn
            if self.verbose:
                port_name = "serial port"
                # Check if self.conn and its attributes still exist before trying to access them
                if (
                    hasattr(self, "conn")
                    and self.conn is not None
                    and hasattr(self.conn, "serial_conn")
                    and self.conn.serial_conn
                    is not None  # Ensure serial_conn itself is not None
                    and hasattr(self.conn.serial_conn, "name")
                ):
                    port_name = self.conn.serial_conn.name
                print(f"pydobot: {port_name} closed")
        finally:
            self.lock.release()

    def __del__(self) -> None:
        """
        Destructor that ensures the connection is closed when the object is
        deleted.
        """
        if self._on:  # Ensure close is only called if not already explicitly closed
            self.close()

    def _initialize_robot(self) -> None:
        """
        Initializes the robot with default parameters upon connection,
        including clearing the command queue, setting PTP joint, coordinate,
        jump, and common parameters, and getting the initial pose.
        """
        self.set_queued_cmd_start_exec()
        self.set_queued_cmd_clear()
        # These parameter settings were implicitly queued in the original code by using ControlValues.Both.
        # To maintain that behavior, is_queued=True is now explicitly passed.
        self.set_ptp_joint_params(
            tagPTPJointParams(
                velocity=[200, 200, 200, 200], acceleration=[200, 200, 200, 200]
            ),
            is_queued=True,
        )
        self.set_ptp_coordinate_params(
            tagPTPCoordinateParams(
                xyzVelocity=200, rVelocity=200, xyzAcceleration=200, rAcceleration=200
            ),
            is_queued=True,
        )
        self.set_ptp_jump_params(
            tagPTPJumpParams(jumpHeight=10, zLimit=200), is_queued=True
        )
        self.set_ptp_common_params(
            tagPTPCommonParams(velocityRatio=100, accelerationRatio=100), is_queued=True
        )
        self.get_pose()

    def _send_command_with_params(
        self,
        command_id: CommunicationProtocolIDs,
        control_value: ControlValues,
        params: Optional[bytes] = None,
        wait: bool = False,
        put_on_queue: bool = False,
    ) -> Message:
        """
        Helper method to construct and send a command message.

        Args:
            command_id: The ID of the command.
            control_value: The base control value (e.g., ReadWrite for set, Zero for get).
            params: Optional byte string of parameters.
            wait: If True and command is queued, waits for execution.
            put_on_queue: If True, command is added to Dobot's queue.

        Returns:
            The response message from the Dobot.
        """
        msg = Message()
        msg.id = command_id

        if put_on_queue:
            if control_value == ControlValues.Zero:  # Get operation, queued
                msg.ctrl = ControlValues.isQueued  # rw=0, Q=1
            else:  # Typically a Write operation, queued
                msg.ctrl = ControlValues.Both  # rw=1, Q=1
        else:
            msg.ctrl = control_value  # Immediate command

        if params is not None:
            msg.params = bytearray(params)
        else:
            msg.params = bytearray([])

        if self.verbose:
            print(f"pydobot: sending from {command_id.name}: {msg}")
        return self._send_command(msg, wait)

    def get_queued_cmd_current_index(self) -> int:
        """
        Retrieves the current index of the command being executed in the queue.
        The Dobot protocol specifies this index is a 64-bit unsigned integer.

        Returns:
            The current command index (int).
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_QUEUED_CMD_CURRENT_INDEX,  # ID 246
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # PDF Table 212 states: uint64_t queuedCmdCurrentIndex
        if not response.params or len(response.params) < 8:
            warnings.warn(
                "Failed to get a valid queuedCmdCurrentIndex."
                + f" Params: {response.params.hex()}"
            )
            raise ValueError(
                "Invalid response for GetQueuedCmdCurrentIndex: insufficient data"
            )
        idx = struct.unpack_from("<Q", response.params, 0)[0]
        return int(idx)

    def get_pose(self) -> tagPose:
        """
        Gets the real-time pose (position and joint angles) of the Dobot.

        Returns:
            tagPose: Pose data.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_POSE,  # ID 10
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        unpacked_response = tagPose.unpack(response.params)

        if self.verbose:
            print(
                "pydobot: x:%03.1f y:%03.1f z:%03.1f r:%03.1f j1:%03.1f j2:%03.1f j3:%03.1f j4:%03.1f"
                % (
                    unpacked_response.x,
                    unpacked_response.y,
                    unpacked_response.z,
                    unpacked_response.r,
                    unpacked_response.jointAngle[0],
                    unpacked_response.jointAngle[1],
                    unpacked_response.jointAngle[2],
                    unpacked_response.jointAngle[3],
                )
            )
        return unpacked_response

    def _read_message(self) -> Optional[Message]:
        """
        Reads a message from the serial connection.

        Returns:
            The received Message object, or None if no message is read.
        """
        time.sleep(0.05)  # Allow data to arrive
        byte_buffer = self.conn.serial_conn.read_all()
        if len(byte_buffer) > 0:
            msg = Message(byte_buffer)
            if self.verbose:
                print("pydobot: <<", msg)
            return msg
        return None

    def _send_command(self, msg: Message, wait: bool = False) -> Message:
        """
        Sends a message to the Dobot and optionally waits for its execution if queued.

        Args:
            msg: The message object to send. msg.ctrl indicates if queued.
            wait: If True AND command was queued, waits for execution.

        Returns:
            The response message from the Dobot.

        Raises:
            TypeError: If no response is received.
            ValueError: If waiting for a queued command and response is malformed.
        """
        self.lock.acquire()
        try:
            self._send_message(msg)
            response_wait_timeout = time.time() + 0.5  # 0.5s for response
            response = None
            while time.time() < response_wait_timeout:
                response = self._read_message()
                if response:
                    break
                time.sleep(0.02)  # Polling interval for response
        finally:
            self.lock.release()

        if response is None:
            raise TypeError(
                f"No response from Dobot. Sent msg ID {msg.id.name if msg.id else 'N/A'}, ctrl {msg.ctrl}"
            )

        # Check if the command sent was intended to be queued (isQueued bit is set)
        is_command_queued_by_sender = (
            msg.ctrl.value & ControlValues.isQueued.value
        ) != 0

        if not wait or not is_command_queued_by_sender:
            return response

        # If wait=True and command was queued, response.params should contain queuedCmdIndex (uint64_t)
        if not response.params or len(response.params) < 8:
            warnings.warn(
                f"Command {msg.id.name} was queued (wait=True), but response.params "
                f"is too short for a 64-bit index. Response: {response}. "
                "Command may have failed to queue properly."
            )
            return response  # Return response for potential inspection

        expected_idx = struct.unpack_from("<Q", response.params, 0)[0]
        if self.verbose:
            print(
                f"pydobot: waiting for command index {expected_idx} (sent {msg.id.name})"
            )

        wait_timeout_seconds = 10  # Max time to wait for a single command execution
        start_wait_time = time.time()

        while True:
            if time.time() - start_wait_time > wait_timeout_seconds:
                warnings.warn(
                    f"Timeout waiting for command index {expected_idx} ({msg.id.name}) to execute."
                )
                break

            try:
                current_idx = self.get_queued_cmd_current_index()
            except (
                ValueError
            ) as e:  # Catch potential errors from get_queued_cmd_current_index
                warnings.warn(
                    f"Error getting current_idx while waiting for {expected_idx}: {e}"
                )
                time.sleep(0.5)  # Wait a bit longer before retrying
                continue

            if current_idx >= expected_idx:
                if self.verbose:
                    print(
                        f"pydobot: command {expected_idx} ({msg.id.name}) executed (current index: {current_idx})."
                    )
                break
            time.sleep(0.1)  # Polling interval for current index

        return response

    def _send_message(self, msg: Message) -> None:
        """
        Writes a message to the Dobot's serial connection.

        Args:
            msg (Message): The message object to send.
        """
        time.sleep(0.05)  # Short delay before writing
        if self.verbose:
            print("pydobot: >>", msg)
        self.conn.serial_conn.write(msg.bytes())

    def set_cp_cmd(
        self, cmd: tagCPCmd, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Executes a CP (Continuous Path) command.
        Protocol ID: 91. Can be queued.

        Args:
            cmd: CP command parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_CP_CMD,  # ID 91
            ControlValues.ReadWrite,
            cmd.pack(),
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_CP_CMD queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_end_effector_gripper(
        self, enable: bool = False, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets the status of the gripper.
        Protocol ID: 63. Can be queued.

        Args:
            enable: True to enable (grip), False to disable (release).
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        # Param: uint8_t isCtrlEnable (always 1 for set), uint8_t isGripped
        params_payload = bytearray([0x01, (0x01 if enable else 0x00)])
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_END_EFFECTOR_GRIPPER,  # ID 63
            ControlValues.ReadWrite,
            params_payload,
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_END_EFFECTOR_GRIPPER queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_end_effector_suction_cup(
        self, enable: bool = False, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets the status of the suction cup.
        Protocol ID: 62. Can be queued.

        Args:
            enable: True to turn on suction, False to turn off.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        # Param: uint8_t isCtrlEnabled (always 1 for set), uint8_t isSucked
        params_payload = bytearray([0x01, (0x01 if enable else 0x00)])
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_END_EFFECTOR_SUCTION_CUP,  # ID 62
            ControlValues.ReadWrite,
            params_payload,
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_END_EFFECTOR_SUCTION_CUP queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_ptp_joint_params(
        self, params: tagPTPJointParams, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets velocity/acceleration for joints in PTP mode.
        Protocol ID: 80. Can be queued.

        Args:
            params: Joint PTP parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_JOINT_PARAMS,  # ID 80
            ControlValues.ReadWrite,
            params.pack(),
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_PTP_JOINT_PARAMS queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_ptp_coordinate_params(
        self,
        params: tagPTPCoordinateParams,
        wait: bool = False,
        is_queued: bool = False,
    ) -> Optional[int]:
        """
        Sets velocity/acceleration of Cartesian axes in PTP mode.
        Protocol ID: 81. Can be queued.

        Args:
            params: Coordinate PTP parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_COORDINATE_PARAMS,  # ID 81
            ControlValues.ReadWrite,
            params.pack(),
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_PTP_COORDINATE_PARAMS queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_ptp_jump_params(
        self, params: tagPTPJumpParams, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets lifting height for JUMP mode in PTP.
        Protocol ID: 82. Can be queued.

        Args:
            params: Jump PTP parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_JUMP_PARAMS,  # ID 82
            ControlValues.ReadWrite,
            params.pack(),
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_PTP_JUMP_PARAMS queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_ptp_common_params(
        self, params: tagPTPCommonParams, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets common velocity/acceleration ratios for PTP mode.
        Protocol ID: 83. Can be queued.

        Args:
            params: Common PTP parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_COMMON_PARAMS,  # ID 83
            ControlValues.ReadWrite,
            params.pack(),
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_PTP_COMMON_PARAMS queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_ptp_cmd(
        self, cmd: tagPTPCmd, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Executes a PTP (Point-to-Point) movement command.
        Protocol ID: 84. Can be queued.

        Args:
            cmd: PTP command parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_PTP_CMD,  # ID 84
            ControlValues.ReadWrite,
            cmd.pack(),
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_PTP_CMD queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_queued_cmd_clear(self) -> Message:
        """
        Clears the command queue. Immediate command.
        Protocol ID: 245.

        Returns:
            The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_QUEUED_CMD_CLEAR,  # ID 245
            ControlValues.ReadWrite,  # rw=1, isQueued=0
        )

    def set_queued_cmd_start_exec(self) -> Message:
        """
        Starts execution of commands in the queue. Immediate command.
        Protocol ID: 240.

        Returns:
            The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_QUEUED_CMD_START_EXEC,  # ID 240
            ControlValues.ReadWrite,  # rw=1, isQueued=0
        )

    def set_wait_cmd(
        self, params: tagWAITCmd, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Adds a wait command to the queue.
        Protocol ID: 110. Can be queued.

        Args:
            params: Wait command parameters (timeout in ms).
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_WAIT_CMD,  # ID 110
            ControlValues.ReadWrite,
            params.pack(),
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_WAIT_CMD queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_queued_cmd_stop_exec(self) -> Message:
        """
        Stops execution of commands in the queue. Immediate command.
        Protocol ID: 241.

        Returns:
            The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_QUEUED_CMD_STOP_EXEC,  # ID 241
            ControlValues.ReadWrite,  # rw=1, isQueued=0
        )

    def set_device_sn(self, device_serial_number: str) -> None:
        """
        Sets the device serial number. Immediate command.
        Protocol ID: 0.

        Args:
            device_serial_number: The serial number to set.
        """
        params_payload = bytearray(device_serial_number.encode("utf-8"))
        self._send_command_with_params(
            CommunicationProtocolIDs.GET_SET_DEVICE_SN,  # ID 0
            ControlValues.ReadWrite,  # rw=1, isQueued=0
            params_payload,
        )
        # Set operations usually don't return specific data beyond ACK, handled by _send_command.
        return None

    def get_device_sn(self) -> str:
        """
        Gets the device serial number. Immediate command.
        Protocol ID: 0.

        Returns:
            Device Serial Number as a string.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_SET_DEVICE_SN,  # ID 0
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return response.params.decode("utf-8")

    def set_device_name(self, device_name: str) -> None:
        """
        Sets the device name. Immediate command.
        Protocol ID: 1.

        Args:
            device_name: The name to set.
        """
        params_payload = bytearray(device_name.encode("utf-8"))
        self._send_command_with_params(
            CommunicationProtocolIDs.GET_SET_DEVICE_NAME,  # ID 1
            ControlValues.ReadWrite,  # rw=1, isQueued=0
            params_payload,
        )
        return None

    def get_device_name(self) -> str:
        """
        Gets the device name. Immediate command.
        Protocol ID: 1.

        Returns:
            Device Name as a string.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_SET_DEVICE_NAME,  # ID 1
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return response.params.decode("utf-8")

    def get_device_version(self) -> Tuple[int, int, int]:
        """
        Gets the device firmware version. Immediate command.
        Protocol ID: 2.

        Returns:
            Tuple (Major version, Minor version, Revision version).
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_DEVICE_VERSION,  # ID 2
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Expected format: uint8_t majorVersion, uint8_t minorVersion, uint8_t revision
        if len(response.params) < 3:
            raise ValueError("Invalid response for GetDeviceVersion: insufficient data")
        major, minor, revision = struct.unpack_from("<BBB", response.params)
        return (major, minor, revision)

    def set_device_rail_capability(self, params: tagWithL) -> None:
        """
        Sets the device's rail capability. Immediate command.
        Protocol ID: 3.

        Args:
            params: Rail version and enable status.
        """
        self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_DEVICE_WITH_RAIL,  # ID 3
            ControlValues.ReadWrite,  # rw=1, isQueued=0
            params.pack(),
        )
        return None

    def get_device_rail_capability(self) -> bool:
        """
        Gets device's rail capability status. Immediate command.
        Protocol ID: 3.

        Returns:
            True if rail enabled/present, False otherwise.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_DEVICE_WITH_RAIL,  # ID 3
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Expected format: uint8_t WithL
        if len(response.params) < 1:
            raise ValueError(
                "Invalid response for GetDeviceRailCapability: insufficient data"
            )
        capability_byte: int = struct.unpack_from("<B", response.params)[0]
        return bool(capability_byte)

    def get_device_time(self) -> int:
        """
        Gets the device's internal time (system tick). Immediate command.
        Protocol ID: 4.

        Returns:
            System Tick (uint32_t).
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_DEVICE_TIME,  # ID 4
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Expected format: uint32_t Time
        if len(response.params) < 4:
            raise ValueError("Invalid response for GetDeviceTime: insufficient data")
        sys_time: int = struct.unpack_from("<I", response.params)[0]
        return sys_time

    def get_device_id(self) -> Tuple[int, int, int]:
        """
        Gets the device ID. Immediate command.
        Protocol uses ID 4 (enum GET_DEVICE_ID) in some docs, but typically distinct from GetDeviceTime.
        Relies on CommunicationProtocolIDs.GET_DEVICE_ID being correct. [Similar to cite: 336 context]

        Returns:
            Device ID as a tuple of 3 uint32_t values.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_DEVICE_ID,  # Assumed ID 5 if GET_DEVICE_TIME is 4
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Expected format: uint32_t[3] ID
        if len(response.params) < 12:  # 3 * 4 bytes
            raise ValueError("Invalid response for GetDeviceID: insufficient data")
        dev_id = struct.unpack_from("<III", response.params)
        return (dev_id[0], dev_id[1], dev_id[2])

    def reset_pose(
        self, manual: int, rear_arm_angle: float, front_arm_angle: float
    ) -> None:
        """
        Resets the real-time pose of the robot. Immediate command.
        Protocol ID: 11.

        Args:
            manual: Manual reset flag (0=auto, 1=manual with angles).
            rear_arm_angle: Rear arm angle for reset (if manual=1).
            front_arm_angle: Front arm angle for reset (if manual=1).
        """
        params_payload = bytearray(
            struct.pack("<Bff", manual, rear_arm_angle, front_arm_angle)
        )
        self._send_command_with_params(
            CommunicationProtocolIDs.RESET_POSE,  # ID 11
            ControlValues.ReadWrite,  # rw=1, isQueued=0
            params_payload,
        )
        return None

    def get_pose_rail(self) -> float:
        """
        Gets the rail pose (position of sliding rail). Immediate command.
        Protocol ID: 13.

        Returns:
            Position of rail (float).
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_POSE_L,  # ID 13
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Expected format: float PoseL
        if len(response.params) < 4:
            raise ValueError("Invalid response for GetPoseRail: insufficient data")
        pose_l: float = struct.unpack_from("<f", response.params)[0]
        if self.verbose:
            print(f"pydobot: l:{pose_l}")
        return pose_l

    def get_alarms_state(self) -> List[int]:
        """
        Gets the current alarm state of the Dobot. Immediate command.
        Protocol ID: 20.

        Returns:
            List of 16 bytes representing alarm states.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_ALARMS_STATE_OR_CLEAR_ALARM,  # ID 20
            ControlValues.Zero,  # For Get: rw=0, isQueued=0
        )
        # Expected format: uint8_t[16] alarmsState
        if len(response.params) < 16:
            raise ValueError("Invalid response for GetAlarmsState: insufficient data")
        return list(struct.unpack_from("<16B", response.params))

    def clear_all_alarms_state(self) -> None:
        """
        Clears all alarm states of the Dobot. Immediate command.
        Protocol ID: 20 (rw=1). (Note: PDF Table 29 shows ID 21 for Clear, assuming ID 20 with rw=1)
        """
        self._send_command_with_params(
            CommunicationProtocolIDs.GET_ALARMS_STATE_OR_CLEAR_ALARM,  # ID 20
            ControlValues.ReadWrite,  # For Clear: rw=1, isQueued=0
        )
        return None

    def set_home_params(
        self, params: tagHomeParams, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets homing parameters (target coordinates).
        Protocol ID: 30. Can be queued.

        Args:
            params: Homing parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_HOME_PARAMS,  # ID 30
            ControlValues.ReadWrite,
            params.pack(),
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_HOME_PARAMS queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def get_home_params(self) -> tagHomeParams:
        """
        Gets homing parameters. Immediate command.
        Protocol ID: 30.

        Returns:
            Current homing parameters.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_HOME_PARAMS,  # ID 30
            ControlValues.Zero,  # For Get: rw=0, isQueued=0
        )
        return tagHomeParams.unpack(response.params)  #

    def set_home_cmd(
        self, params: tagHomeCmd, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Executes the homing function.
        Protocol ID: 31. Can be queued.

        Args:
            params: Homing command options (reserved).
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_HOME_CMD,  # ID 31
            ControlValues.ReadWrite,
            params.pack(),  # HOMECmd
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_HOME_CMD queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_autoleveling(
        self,
        params: tagAutoLevelingParams,
        wait: bool = False,
        is_queued: bool = False,
    ) -> Optional[int]:
        """
        Sets auto-leveling parameters and initiates auto-leveling.
        Protocol ID: 32. Can be queued.

        Args:
            params: Auto-leveling parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_AUTO_LEVELING,  # ID 32
            ControlValues.ReadWrite,
            params.pack(),  # AutoLevelingParams
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_AUTO_LEVELING queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def get_autoleveling(self) -> float:
        """
        Gets automatic leveling result/status. Immediate command.
        Protocol ID: 32.

        Returns:
            AutoLevelingResult (float, accuracy or status).
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_AUTO_LEVELING,  # ID 32
            ControlValues.Zero,  # For Get: rw=0, isQueued=0
        )
        # Expected: float AutoLevelingResult
        if not response.params or len(response.params) < 4:
            raise ValueError("Invalid response for GetAutoLeveling: insufficient data")
        return float(struct.unpack_from("<f", response.params)[0])

    def set_hht_trig_mode(self, mode: HHTTrigMode) -> None:
        """
        Sets Hand Hold Teaching trigger mode. Immediate command.
        Protocol ID: 40.

        Args:
            mode: The HHT trigger mode.

        """
        params_payload = bytearray([mode.value])
        self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_HHTTRIG_MODE,  # ID 40
            ControlValues.ReadWrite,  # rw=1, isQueued=0
            params_payload,
        )

    def get_hht_trig_mode(self) -> HHTTrigMode:
        """
        Gets Hand Hold Teaching trigger mode. Immediate command.
        Protocol ID: 40.

        Returns:
            Current HHTTrigMode.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_HHTTRIG_MODE,  # ID 40
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Expected: HHTTrigMode (uint8_t)
        if not response.params or len(response.params) < 1:
            raise ValueError("Invalid response for GetHHTTrigMode: insufficient data")
        return HHTTrigMode(struct.unpack_from("<B", response.params)[0])

    def set_hht_trig_output_enabled(self, is_enabled: bool) -> None:
        """
        Enables/disables Hand Hold Teaching trigger output. Immediate command.
        Protocol ID: 41.

        Args:
            is_enabled: True to enable, False to disable.

        """
        params_payload = bytearray([1 if is_enabled else 0])
        self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_HHTTRIG_OUTPUT_ENABLED,  # ID 41
            ControlValues.ReadWrite,  # rw=1, isQueued=0
            params_payload,
        )
        return None

    def get_hht_trig_output_enabled(self) -> bool:
        """
        Checks if Hand Hold Teaching trigger output is enabled. Immediate command.
        Protocol ID: 41.

        Returns:
            True if enabled, False otherwise.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_HHTTRIG_OUTPUT_ENABLED,  # ID 41
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Expected: uint8_t isEnabled
        if not response.params or len(response.params) < 1:
            raise ValueError(
                "Invalid response for GetHHTTrigOutputEnabled: insufficient data"
            )
        return bool(struct.unpack_from("<B", response.params)[0])

    def get_hht_trig_output(self) -> bool:
        """
        Gets current Hand Hold Teaching trigger output value. Immediate command.
        Protocol ID: 42.

        Returns:
            True if triggered, False otherwise.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_HHTTRIG_OUTPUT,  # ID 42
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Expected: uint8_t isTriggered
        if not response.params or len(response.params) < 1:
            raise ValueError("Invalid response for GetHHTTrigOutput: insufficient data")
        return bool(struct.unpack_from("<B", response.params)[0])

    def set_end_effector_params(
        self, params: tagEndEffectorParams, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets parameters (bias) for the end effector.
        Protocol ID: 60. Can be queued.

        Args:
            params: End effector parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_END_EFFECTOR_PARAMS,  # ID 60
            ControlValues.ReadWrite,
            params.pack(),  # EndEffectorParams
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_END_EFFECTOR_PARAMS queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def get_end_effector_params(self) -> tagEndEffectorParams:
        """
        Gets parameters (bias) for the end effector. Immediate command.
        Protocol ID: 60.

        Returns:
            Current end effector parameters.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_END_EFFECTOR_PARAMS,  # ID 60
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagEndEffectorParams.unpack(response.params)  #

    def set_end_effector_laser(
        self, enable_ctrl: bool, on: bool, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Controls the laser end effector.
        Protocol ID: 61. Can be queued.

        Args:
            enable_ctrl: True to enable laser control (typically True for setting).
            on: True to turn laser on, False to turn off.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        params_payload = bytearray([1 if enable_ctrl else 0, 1 if on else 0])
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_END_EFFECTOR_LASER,  # ID 61
            ControlValues.ReadWrite,
            params_payload,
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_END_EFFECTOR_LASER queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def get_end_effector_laser(self) -> Tuple[bool, bool]:
        """
        Gets status of the laser end effector. Immediate command.
        Protocol ID: 61.

        Returns:
            Tuple (isCtrlEnabled, isOn).
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_END_EFFECTOR_LASER,  # ID 61
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Expected: uint8_t isCtrlEnabled, uint8_t isOn
        if not response.params or len(response.params) < 2:
            raise ValueError(
                "Invalid response for GetEndEffectorLaser: insufficient data"
            )
        is_ctrl_enabled = bool(struct.unpack_from("<B", response.params, 0)[0])
        is_on = bool(struct.unpack_from("<B", response.params, 1)[0])
        return is_ctrl_enabled, is_on

    def get_end_effector_suction_cup(self) -> Tuple[bool, bool]:
        """
        Gets status of the suction cup. Immediate command.
        Protocol ID: 62.

        Returns:
            Tuple (isCtrlEnabled, isSucked).
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_END_EFFECTOR_SUCTION_CUP,  # ID 62
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Expected: uint8_t isCtrlEnable, uint8_t isSucked
        if not response.params or len(response.params) < 2:
            raise ValueError(
                "Invalid response for GetEndEffectorSuctionCup: insufficient data"
            )
        is_ctrl_enabled = bool(struct.unpack_from("<B", response.params, 0)[0])
        is_sucked = bool(struct.unpack_from("<B", response.params, 1)[0])
        return is_ctrl_enabled, is_sucked

    def get_end_effector_gripper(self) -> Tuple[bool, bool]:
        """
        Gets status of the gripper. Immediate command.
        Protocol ID: 63.

        Returns:
            Tuple (isCtrlEnabled, isGripped).
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_END_EFFECTOR_GRIPPER,  # ID 63
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Expected: uint8_t isCtrlEnable, uint8_t isGripped
        if not response.params or len(response.params) < 2:
            raise ValueError(
                "Invalid response for GetEndEffectorGripper: insufficient data"
            )
        is_ctrl_enabled = bool(struct.unpack_from("<B", response.params, 0)[0])
        is_gripped = bool(struct.unpack_from("<B", response.params, 1)[0])
        return is_ctrl_enabled, is_gripped

    def set_jog_joint_params(
        self, params: tagJOGJointParams, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets parameters for joint mode JOG movements.
        Protocol ID: 70. Can be queued.

        Args:
            params: JOG joint parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_JOG_JOINT_PARAMS,  # ID 70
            ControlValues.ReadWrite,
            params.pack(),  # JOGJointParams
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_JOG_JOINT_PARAMS queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def get_jog_joint_params(self) -> tagJOGJointParams:
        """
        Gets parameters for joint mode JOG movements. Immediate command.
        Protocol ID: 70.

        Returns:
            Current JOG joint parameters.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_JOG_JOINT_PARAMS,  # ID 70
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagJOGJointParams.unpack(response.params)  #

    def set_jog_coordinate_params(
        self,
        params: tagJOGCoordinateParams,
        wait: bool = False,
        is_queued: bool = False,
    ) -> Optional[int]:
        """
        Sets parameters for coordinate mode JOG movements.
        Protocol ID: 71. Can be queued.

        Args:
            params: JOG coordinate parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_JOG_COORDINATE_PARAMS,  # ID 71
            ControlValues.ReadWrite,
            params.pack(),  # JOGCoordinateParams
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_JOG_COORDINATE_PARAMS queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def get_jog_coordinate_params(self) -> tagJOGCoordinateParams:
        """
        Gets parameters for coordinate mode JOG movements. Immediate command.
        Protocol ID: 71.

        Returns:
            Current JOG coordinate parameters.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_JOG_COORDINATE_PARAMS,  # ID 71
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagJOGCoordinateParams.unpack(response.params)  #

    def set_jog_common_params(
        self, params: tagJOGCommonParams, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets common JOG parameters (velocity/acceleration ratios).
        Protocol ID: 72. Can be queued.

        Args:
            params: Common JOG parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_JOG_COMMON_PARAMS,  # ID 72
            ControlValues.ReadWrite,
            params.pack(),  # JOGCommonParams
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_JOG_COMMON_PARAMS queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def get_jog_common_params(self) -> tagJOGCommonParams:
        """
        Gets common JOG parameters. Immediate command.
        Protocol ID: 72.

        Returns:
            Current common JOG parameters.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_JOG_COMMON_PARAMS,  # ID 72
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagJOGCommonParams.unpack(response.params)  #

    def set_jog_cmd(
        self, cmd: tagJOGCmd, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Executes a JOG command.
        Protocol ID: 73. Can be queued.

        Args:
            cmd: JOG command parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_JOG_CMD,  # ID 73
            ControlValues.ReadWrite,
            cmd.pack(),  # JOGCmd
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_JOG_CMD queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_jogl_params(
        self, params: tagJOGLParams, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets parameters for JOGL (linear jog) mode.
        Protocol ID: 74. Can be queued.

        Args:
            params: JOGL parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_JOGL_PARAMS,  # ID 74
            ControlValues.ReadWrite,
            params.pack(),  # JOGLParams
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_JOGL_PARAMS queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def get_jogl_params(self) -> tagJOGLParams:
        """
        Gets parameters for JOGL (linear jog) mode. Immediate command.
        Protocol ID: 74.

        Returns:
            Current JOGL parameters.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_JOGL_PARAMS,  # ID 74
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagJOGLParams.unpack(response.params)  #

    def get_ptp_joint_params(self) -> tagPTPJointParams:
        """
        Gets PTP joint parameters. Immediate command.
        Protocol ID: 80.

        Returns:
            Current PTP joint parameters.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_JOINT_PARAMS,  # ID 80
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagPTPJointParams.unpack(response.params)  # structure, 395 for get table

    def get_ptp_coordinate_params(self) -> tagPTPCoordinateParams:
        """
        Gets PTP coordinate parameters. Immediate command.
        Protocol ID: 81.

        Returns:
            Current PTP coordinate parameters.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_COORDINATE_PARAMS,  # ID 81
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagPTPCoordinateParams.unpack(
            response.params
        )  # structure, 399 for get table

    def get_ptp_jump_params(self) -> tagPTPJumpParams:
        """
        Gets PTP jump parameters. Immediate command.
        Protocol ID: 82.

        Returns:
            Current PTP jump parameters.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_JUMP_PARAMS,  # ID 82
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagPTPJumpParams.unpack(response.params)  # structure, 403 for get table

    def get_ptp_common_params(self) -> tagPTPCommonParams:
        """
        Gets PTP common parameters. Immediate command.
        Protocol ID: 83.

        Returns:
            Current PTP common parameters.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_COMMON_PARAMS,  # ID 83
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagPTPCommonParams.unpack(
            response.params
        )  # structure, 408 for get table

    def set_ptpl_params(
        self, params: tagPTPLParams, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets parameters for PTPL (Point-to-Point Linear) mode.
        Protocol ID: 85. Can be queued.

        Args:
            params: PTPL parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTPL_PARAMS,  # ID 85
            ControlValues.ReadWrite,
            params.pack(),  # PTPLParams
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_PTPL_PARAMS queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def get_ptpl_params(self) -> tagPTPLParams:
        """
        Gets parameters for PTPL mode. Immediate command.
        Protocol ID: 85.

        Returns:
            Current PTPL parameters.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTPL_PARAMS,  # ID 85
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagPTPLParams.unpack(response.params)  #

    def set_ptp_with_rail_cmd(
        self, cmd: tagPTPWithLCmd, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Executes a PTP command with rail movement.
        Protocol ID: 86. Can be queued.

        Args:
            cmd: PTP command with rail parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_PTP_WITH_L_CMD,  # ID 86
            ControlValues.ReadWrite,
            cmd.pack(),  # PTPWithLCmd
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_PTP_WITH_L_CMD queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_ptp_jump2_params(
        self, params: tagPTPJump2Params, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets extended jump parameters for PTP movements.
        Protocol ID: 87. Can be queued.

        Args:
            params: PTP jump2 parameters (start/end jump heights).
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_JUMP_TO_PARAMS,  # ID 87
            ControlValues.ReadWrite,
            params.pack(),  # PTPJump2Params
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_PTP_JUMP_TO_PARAMS queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def get_ptp_jump2_params(self) -> tagPTPJump2Params:
        """
        Gets extended jump parameters for PTP movements. Immediate command.
        Protocol ID: 87.

        Returns:
            Current PTP jump2 parameters.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_JUMP_TO_PARAMS,  # ID 87
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagPTPJump2Params.unpack(response.params)  #

    def set_ptp_po_cmd(
        self,
        ptp_cmd: tagPTPCmd,
        po_cmds: List[tagPOCmd],
        wait: bool = False,
        is_queued: bool = False,
    ) -> Optional[int]:
        """
        Executes a PTP command with multiple PO (Point Output) commands.
        Protocol ID: 88. Can be queued.

        Args:
            ptp_cmd: The PTP command.
            po_cmds: A list of PO commands.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        params_bytes = bytearray(ptp_cmd.pack())  # PTPCmd
        for po_cmd in po_cmds:
            params_bytes.extend(po_cmd.pack())  # POCmd

        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_PTPPO_CMD,  # ID 88
            ControlValues.ReadWrite,
            bytes(params_bytes),
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_PTPPO_CMD queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_ptp_po_with_rail_cmd(
        self,
        ptp_cmd: tagPTPWithLCmd,
        po_cmds: List[tagPOCmd],
        wait: bool = False,
        is_queued: bool = False,
    ) -> Optional[int]:
        """
        Executes a PTP command with rail and PO commands.
        Protocol ID: 89. Can be queued.

        Args:
            ptp_cmd: PTP command with rail.
            po_cmds: A list of PO commands.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        params_bytes = bytearray(ptp_cmd.pack())  # PTPWithLCmd
        for po_cmd in po_cmds:
            params_bytes.extend(po_cmd.pack())  # POCmd

        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_PTPPO_WITH_L_CMD,  # ID 89
            ControlValues.ReadWrite,
            bytes(params_bytes),
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_PTPPO_WITH_L_CMD queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_cp_params(
        self, params: tagCPParams, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets parameters for CP (Continuous Path) movements.
        Protocol ID: 90. Can be queued.

        Args:
            params: CP parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_CP_PARAMS,  # ID 90
            ControlValues.ReadWrite,
            params.pack(),  # CPParams
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_CP_PARAMS queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def get_cp_params(self) -> tagCPParams:
        """
        Gets parameters for CP movements. Immediate command.
        Protocol ID: 90.

        Returns:
            Current CP parameters.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_CP_PARAMS,  # ID 90
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagCPParams.unpack(response.params)  #

    def set_cp_le_cmd(
        self, cmd: tagCPCmd, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Executes a CP command for laser engraving.
        Protocol ID: 92. Can be queued.

        Args:
            cmd: CP command (used for laser engraving context).
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_CPLE_CMD,  # ID 92
            ControlValues.ReadWrite,
            cmd.pack(),  # CPCmd
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_CPLE_CMD queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_arc_params(
        self, params: tagARCParams, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets parameters for ARC (Arc) movements.
        Protocol ID: 100. Can be queued.

        Args:
            params: ARC parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_ARC_PARAMS,  # ID 100
            ControlValues.ReadWrite,
            params.pack(),  # ARCParams
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_ARC_PARAMS queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def get_arc_params(self) -> tagARCParams:
        """
        Gets parameters for ARC movements. Immediate command.
        Protocol ID: 100.

        Returns:
            Current ARC parameters.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_ARC_PARAMS,  # ID 100
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagARCParams.unpack(response.params)  #

    def set_arc_cmd(
        self, cmd: tagARCCmd, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Executes an ARC (Arc) movement command.
        Protocol ID: 101. Can be queued.

        Args:
            cmd: ARC command parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_ARC_CMD,  # ID 101
            ControlValues.ReadWrite,
            cmd.pack(),  # ARCCmd
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_ARC_CMD queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_trig_cmd(
        self, cmd: tagTRIGCmd, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Executes a TRIG (Trigger) command.
        Protocol ID: 120. Can be queued.

        Args:
            cmd: TRIG command parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_TRIG_CMD,  # ID 120
            ControlValues.ReadWrite,
            cmd.pack(),  # TRIGCmd
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_TRIG_CMD queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_io_multiplexing(
        self, params: tagIOMultiplexing, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets I/O multiplexing configuration.
        Protocol ID: 130. Can be queued.

        Args:
            params: I/O multiplexing parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_IO_MULTIPLEXING,  # ID 130
            ControlValues.ReadWrite,
            params.pack(),  # IOMultiplexing
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_IO_MULTIPLEXING queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def get_io_multiplexing(self) -> tagIOMultiplexing:
        """
        Gets I/O multiplexing configuration. Immediate command.
        Protocol ID: 130.
        Note: Protocol implies getting config for an address, but Get command takes no address.

        Returns:
            Current I/O multiplexing configuration (likely for a default/first address).
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_IO_MULTIPLEXING,  # ID 130
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagIOMultiplexing.unpack(response.params)  #

    def set_io_do(
        self, params: tagIODO, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets digital output (DO) for a specific I/O.
        Protocol ID: 131. Can be queued.

        Args:
            params: I/O DO parameters (address, level).
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_IODO,  # ID 131
            ControlValues.ReadWrite,
            params.pack(),  # IODO
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_IODO queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def get_io_do(self, address: int) -> Level:  # Address param is for user expectation
        """
        Gets digital output (DO) status for an I/O. Immediate command.
        Protocol ID: 131.
        Note: GetIODO command takes no address param; response contains address and level.

        Args:
            address: The EIO address (1-20) user is interested in. (Not sent to Dobot)

        Returns:
            The Level of the DO. Caller should check response address if specific one is needed.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_IODO,  # ID 131
            ControlValues.Zero,  # For Get: rw=0, isQueued=0
        )
        # Returns IODO (address, level)
        unpacked_response = tagIODO.unpack(response.params)
        if self.verbose and unpacked_response.address != address:
            warnings.warn(
                f"GetIODO: Requested address {address}, response is for {unpacked_response.address}"
            )
        return unpacked_response.level

    def set_io_pwm(
        self, params: tagIOPWM, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets PWM output for a specific I/O.
        Protocol ID: 132. Can be queued.

        Args:
            params: I/O PWM parameters (address, frequency, dutyCycle).
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_IO_PWM,  # ID 132
            ControlValues.ReadWrite,
            params.pack(),  # IOPWM
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_IO_PWM queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def get_io_pwm(
        self, address: int
    ) -> tagIOPWM:  # Address param is for user expectation
        """
        Gets PWM output status for an I/O. Immediate command.
        Protocol ID: 132.
        Note: GetIOPWM command takes no address param; response contains address and PWM config.

        Args:
            address: The EIO address (1-20) user is interested in. (Not sent to Dobot)

        Returns:
            PWM configuration. Caller should check response address.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_IO_PWM,  # ID 132
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Returns IOPWM (address, frequency, dutyCycle)
        unpacked_response = tagIOPWM.unpack(response.params)
        if self.verbose and unpacked_response.address != address:
            warnings.warn(
                f"GetIOPWM: Requested address {address}, response is for {unpacked_response.address}"
            )
        return unpacked_response

    def get_io_di(self, address: int) -> Level:  # Address param is for user expectation
        """
        Gets digital input (DI) status for an I/O. Immediate command.
        Protocol ID: 133.
        Note: GetIODI command takes no address param; response contains address and level.

        Args:
            address: The EIO address (1-20) user is interested in. (Not sent to Dobot)

        Returns:
            The Level of the DI. Caller should check response address.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_IODI,  # ID 133
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Returns IODI (address, level) - struct is same as IODO
        unpacked_response = tagIODO.unpack(response.params)
        if self.verbose and unpacked_response.address != address:
            warnings.warn(
                f"GetIODI: Requested address {address}, response is for {unpacked_response.address}"
            )
        return unpacked_response.level

    def get_io_adc(self, address: int) -> int:  # Address param is for user expectation
        """
        Gets ADC value for an I/O. Immediate command.
        Protocol ID: 134.
        Note: GetIOADC command takes no address param; response contains address and value.

        Args:
            address: The EIO address (1-20) user is interested in. (Not sent to Dobot)

        Returns:
            ADC value (0-4095). Caller should check response address.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_IO_ADC,  # ID 134
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Expected IOADC struct: {uint8_t address; uint16_t value;}
        addr_val_format = "<BH"  # address (1 byte), value (2 bytes)
        expected_len = struct.calcsize(addr_val_format)

        if not response.params or len(response.params) < expected_len:
            raise ValueError(
                f"Invalid response for GetIOADC: insufficient data, expected {expected_len} bytes."
            )

        response_address, value = struct.unpack_from(addr_val_format, response.params)

        if self.verbose:
            print(
                f"pydobot: get_io_adc for requested_address={address}, response_address={response_address}, value={value}"
            )
            if response_address != address:
                warnings.warn(
                    f"GetIOADC: Requested address {address}, but response is for {response_address}"
                )
        return int(value)

    def set_e_motor(
        self, params: tagEMOTOR, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Controls an external motor (stepper).
        Protocol ID: 135. Can be queued.

        Args:
            params: External motor parameters.
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_EMOTOR,  # ID 135
            ControlValues.ReadWrite,
            params.pack(),  # EMotor
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_EMOTOR queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_color_sensor(
        self, params: tagDevice, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets parameters for the color sensor.
        Protocol ID: 137. Can be queued.

        Args:
            params: Device parameters for color sensor (isEnable, port, version).
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_COLOR_SENSOR,  # ID 137
            ControlValues.ReadWrite,
            params.pack(),  # Device ColorSense
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_COLOR_SENSOR queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def get_color_sensor(self) -> Tuple[int, int, int]:
        """
        Gets readings (R, G, B) from the color sensor. Immediate command.
        Protocol ID: 137.

        Returns:
            Tuple (r, g, b) color values (uint8_t each).
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_COLOR_SENSOR,  # ID 137
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Expected: Color (r, g, b) uint8_t each
        if not response.params or len(response.params) < 3:
            raise ValueError("Invalid response for GetColorSensor: insufficient data")
        r, g, b = struct.unpack_from("<BBB", response.params)
        return r, g, b

    def set_ir_switch(
        self, params: tagDevice, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Sets parameters for the IR (Infrared) switch.
        Protocol ID: 138. Can be queued.

        Args:
            params: Device parameters for IR switch (isEnable, port, version).
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_IR_SWITCH,  # ID 138
            ControlValues.ReadWrite,
            params.pack(),  # Device IRSense (reusing Device struct)
            wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_GET_IR_SWITCH queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def get_ir_switch(self) -> bool:
        """
        Gets the status of the IR switch. Immediate command.
        Protocol ID: 138.

        Returns:
            True if IR switch is triggered/active, False otherwise.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_IR_SWITCH,  # ID 138
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Expected: uint8_t state
        if not response.params or len(response.params) < 1:
            raise ValueError("Invalid response for GetIRSwitch: insufficient data")
        return bool(struct.unpack_from("<B", response.params)[0])

    def set_angle_sensor_static_error(
        self, rear_arm_angle_error: float, front_arm_angle_error: float
    ) -> Message:
        """
        Sets static error for angle sensors. Immediate command.
        Protocol ID: 140.

        Args:
            rear_arm_angle_error: Static error for rear arm angle sensor.
            front_arm_angle_error: Static error for front arm angle sensor.

        Returns:
            The response message from the Dobot.
        """
        params_payload = bytearray(
            struct.pack("<ff", rear_arm_angle_error, front_arm_angle_error)
        )
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_ANGLE_SENSOR_STATIC_ERROR,  # ID 140
            ControlValues.ReadWrite,  # rw=1, isQueued=0
            params_payload,
        )

    def get_angle_sensor_static_error(self) -> Tuple[float, float]:
        """
        Gets static error for angle sensors. Immediate command.
        Protocol ID: 140.

        Returns:
            Tuple (rear_arm_angle_error, front_arm_angle_error).
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_ANGLE_SENSOR_STATIC_ERROR,  # ID 140
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Expected: float rearArmAngleError, float frontArmAngleError
        if not response.params or len(response.params) < 8:  # 2 floats
            raise ValueError(
                "Invalid response for GetAngleSensorStaticError: insufficient data"
            )
        rear_err, front_err = struct.unpack_from("<ff", response.params)
        return rear_err, front_err

    def set_wifi_config_mode(self, enable: bool) -> Message:
        """
        Enables or disables Wi-Fi configuration mode. Immediate command.
        Protocol ID: 150.

        Args:
            enable: True to enable, False to disable.

        Returns:
            The response message from the Dobot.
        """
        params_payload = bytearray([1 if enable else 0])
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_CONFIG_MODE,  # ID 150
            ControlValues.ReadWrite,  # rw=1, isQueued=0
            params_payload,
        )

    def get_wifi_config_mode(self) -> bool:
        """
        Gets Wi-Fi configuration mode status. Immediate command.
        Protocol ID: 150.

        Returns:
            True if Wi-Fi config mode enabled, False otherwise.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_CONFIG_MODE,  # ID 150
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Expected: uint8_t enable
        if not response.params or len(response.params) < 1:
            raise ValueError(
                "Invalid response for GetWIFIConfigMode: insufficient data"
            )
        return bool(struct.unpack_from("<B", response.params)[0])

    def set_wifi_ssid(self, ssid: str) -> Message:
        """
        Sets the Wi-Fi SSID. Immediate command.
        Protocol ID: 151.

        Args:
            ssid: The Wi-Fi SSID.

        Returns:
            The response message from the Dobot.
        """
        params_payload = bytearray(ssid.encode("utf-8"))
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_SSID,  # ID 151
            ControlValues.ReadWrite,  # rw=1, isQueued=0
            params_payload,
        )

    def get_wifi_ssid(self) -> str:
        """
        Gets the Wi-Fi SSID. Immediate command.
        Protocol ID: 151.

        Returns:
            Wi-Fi SSID as string.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_SSID,  # ID 151
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return response.params.decode("utf-8")  # Returns char[n] ssid

    def set_wifi_password(self, password: str) -> Message:
        """
        Sets the Wi-Fi password. Immediate command.
        Protocol ID: 152.

        Args:
            password: The Wi-Fi password.

        Returns:
            The response message from the Dobot.
        """
        params_payload = bytearray(password.encode("utf-8"))
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_PASSWORD,  # ID 152
            ControlValues.ReadWrite,  # rw=1, isQueued=0
            params_payload,
        )

    def get_wifi_password(self) -> str:
        """
        Gets the Wi-Fi password. Immediate command.
        Protocol ID: 152.

        Returns:
            Wi-Fi password as string.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_PASSWORD,  # ID 152
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return response.params.decode("utf-8")  # Returns char[n] password

    def set_wifi_ip_address(self, params: tagWIFIIPAddress) -> Message:
        """
        Sets Wi-Fi IP address settings. Immediate command.
        Protocol ID: 153.

        Args:
            params: WIFIIPAddress structure.

        Returns:
            The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_IP_ADDRESS,  # ID 153
            ControlValues.ReadWrite,  # rw=1, isQueued=0
            params.pack(),
        )

    def get_wifi_ip_address(self) -> tagWIFIIPAddress:
        """
        Gets Wi-Fi IP address settings. Immediate command.
        Protocol ID: 153.

        Returns:
            WIFIIPAddress structure.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_IP_ADDRESS,  # ID 153
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagWIFIIPAddress.unpack(response.params)  #

    def set_wifi_netmask(self, params: tagWIFINetmask) -> Message:
        """
        Sets Wi-Fi netmask settings. Immediate command.
        Protocol ID: 154.

        Args:
            params: WIFINetmask structure.

        Returns:
            The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_NETMASK,  # ID 154
            ControlValues.ReadWrite,  # rw=1, isQueued=0
            params.pack(),
        )

    def get_wifi_netmask(self) -> tagWIFINetmask:
        """
        Gets Wi-Fi netmask settings. Immediate command.
        Protocol ID: 154.

        Returns:
            WIFINetmask structure.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_NETMASK,  # ID 154
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagWIFINetmask.unpack(response.params)  #

    def set_wifi_gateway(self, params: tagWIFIGateway) -> Message:
        """
        Sets Wi-Fi gateway settings. Immediate command.
        Protocol ID: 155.

        Args:
            params: WIFIGateway structure.

        Returns:
            The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_GATEWAY,  # ID 155
            ControlValues.ReadWrite,  # rw=1, isQueued=0
            params.pack(),
        )

    def get_wifi_gateway(self) -> tagWIFIGateway:
        """
        Gets Wi-Fi gateway settings. Immediate command.
        Protocol ID: 155.

        Returns:
            WIFIGateway structure.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_GATEWAY,  # ID 155
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagWIFIGateway.unpack(response.params)  #

    def set_wifi_dns(self, params: tagWIFIDNS) -> Message:
        """
        Sets Wi-Fi DNS settings. Immediate command.
        Protocol ID: 156.

        Args:
            params: WIFIDNS structure.

        Returns:
            The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_DNS,  # ID 156
            ControlValues.ReadWrite,  # rw=1, isQueued=0
            params.pack(),
        )

    def get_wifi_dns(self) -> tagWIFIDNS:
        """
        Gets Wi-Fi DNS settings. Immediate command.
        Protocol ID: 156.

        Returns:
            WIFIDNS structure.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_DNS,  # ID 156
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        return tagWIFIDNS.unpack(response.params)  #

    def get_wifi_connect_status(self) -> bool:
        """
        Gets Wi-Fi connection status. Immediate command.
        Protocol ID: 157.

        Returns:
            True if connected, False otherwise.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_WIFI_CONNECT_STATUS,  # ID 157
            ControlValues.Zero,  # rw=0, isQueued=0
        )
        # Expected: uint8_t isConnected
        if not response.params or len(response.params) < 1:
            raise ValueError(
                "Invalid response for GetWIFIConnectStatus: insufficient data"
            )
        return bool(struct.unpack_from("<B", response.params)[0])

    def set_lost_step_params(self, value: float) -> Message:
        """
        Sets parameters for losing-step detection threshold. Immediate command.
        Protocol ID: 170.

        Args:
            value: Threshold value for lost step parameters (float).

        Returns:
            The response message from the Dobot.
        """
        params_payload = bytearray(struct.pack("<f", value))
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_LOST_STEP_PARAMS,  # ID 170
            ControlValues.ReadWrite,  # rw=1, isQueued=0
            params_payload,
        )

    def set_lost_step_cmd(
        self, wait: bool = False, is_queued: bool = False
    ) -> Optional[int]:
        """
        Executes a losing-step detection command.
        Protocol ID: 171. Can be queued.

        Args:
            wait: If True and command is queued, waits for execution.
            is_queued: If True, command is added to the queue.

        Returns:
            Queued command index if is_queued is True, else None.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_LOST_STEP_CMD,  # ID 171
            ControlValues.ReadWrite,
            params=None,  # No specific params for SetLostStepCmd
            wait=wait,
            put_on_queue=is_queued,
        )
        if is_queued:
            if response.params and len(response.params) >= 8:
                return int(struct.unpack("<Q", response.params)[0])  #
            warnings.warn(
                f"SET_LOST_STEP_CMD queued but no valid index returned. Params: {response.params.hex()}"
            )
            return None
        return None

    def set_queued_cmd_force_stop_exec(self) -> Message:
        """
        Forces stop of command execution in queue. Immediate command.
        Protocol ID: 242.

        Returns:
            The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_QUEUED_CMD_FORCE_STOP_EXEC,  # ID 242
            ControlValues.ReadWrite,  # rw=1, isQueued=0
        )

    def set_queued_cmd_start_download(
        self, total_loop: int, line_per_loop: int
    ) -> Message:
        """
        Starts downloading commands to queue for offline execution. Immediate command.
        Protocol ID: 243.

        Args:
            total_loop: Total number of loops for command download.
            line_per_loop: Number of lines per loop for command download.

        Returns:
            The response message from the Dobot.
        """
        params_payload = bytearray(struct.pack("<II", total_loop, line_per_loop))
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_QUEUED_CMD_START_DOWNLOAD,  # ID 243
            ControlValues.ReadWrite,  # rw=1, isQueued=0
            params_payload,
        )

    def set_queued_cmd_stop_download(self) -> Message:
        """
        Stops downloading commands to the queue. Immediate command.
        Protocol ID: 244.

        Returns:
            The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_QUEUED_CMD_STOP_DOWNLOAD,  # ID 244
            ControlValues.ReadWrite,  # rw=1, isQueued=0
        )
