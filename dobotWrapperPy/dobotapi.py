# mypy: disable-error-code="import"
import struct
import threading
import time
import warnings
from typing import Optional, List

from .dobotConnection import DobotConnection
from .enums.CommunicationProtocolIDs import CommunicationProtocolIDs
from .enums.ControlValues import ControlValues
from .enums.HHTTrigMode import HHTTrigMode
from .enums.ptpMode import PTPMode
from .enums.tagVersionRail import tagVersionRail
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
    is_open: bool
    ctrl: ControlValues

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
                else "failed to open serial port"
            )

        self._initialize_robot()

    def close(self) -> None:
        """
        Closes the serial connection to the Dobot and releases the lock.
        """
        self._on = False
        self.lock.acquire()
        if self.verbose:
            print("pydobot: %s closed" % self.conn.serial_conn.name)
        if hasattr(self, "conn") and self.conn is not None:
            del self.conn
        self.lock.release()

    def __del__(self) -> None:
        """
        Destructor that ensures the connection is closed when the object is
        deleted.
        """
        self.close()

    def _initialize_robot(self) -> None:
        """
        Initializes the robot with default parameters upon connection,
        including clearing the command queue, setting PTP joint, coordinate,
        jump, and common parameters, and getting the initial pose.
        """
        self.set_queued_cmd_start_exec()
        self.set_queued_cmd_clear()
        self.set_ptp_joint_params(
            tagPTPJointParams(
                velocity=[200, 200, 200, 200], acceleration=[200, 200, 200, 200]
            )
        )
        self.set_ptp_coordinate_params(tagPTPCoordinateParams(200, 200, 200, 200))
        self.set_ptp_jump_params(tagPTPJumpParams(10, 200))
        self.set_ptp_common_params(tagPTPCommonParams(100, 100))
        self.get_pose()

    def _send_command_with_params(
        self,
        command_id: CommunicationProtocolIDs,
        control_value: ControlValues,
        params: Optional[bytes] = None,
        wait: bool = False,
    ) -> Message:
        """
        Helper method to construct and send a command message with specified
        parameters.

        Args:
            command_id (CommunicationProtocolIDs): The ID of the command to
                send.
            control_value (ControlValues): The control value for the command.
            params: Optional byte string of parameters for
                the command (default: None).
            wait: If True, waits for the command to be executed by the
                robot (default: False).

        Returns:
            Message: The response message from the Dobot.
        """
        msg = Message()
        msg.id = command_id
        msg.ctrl = control_value
        if params is not None:
            msg.params = params
        else:
            msg.params = bytearray([])

        if self.verbose:
            print(f"pydobot: sending from {command_id.name}: {msg}")
        response = self._send_command(msg, wait)
        return response

    def get_queued_cmd_current_index(self) -> int:
        """
        Retrieves the current index of the command queue.

        Returns:
            The current command index (int).
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_QUEUED_CMD_CURRENT_INDEX,
            ControlValues.Zero,
        )
        idx = int(struct.unpack_from("<L", response.params, 0)[0])
        return idx

    def get_pose(self) -> Message:
        """
        Gets the real-time pose (position and joint angles) of the Dobot.

        Returns:
            Message: The response message containing the pose data.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_POSE, ControlValues.Zero
        )
        unpacked_response = tagPose.unpack(response.params)

        if self.verbose:
            print(
                "pydobot: x:%03.1f \
                            y:%03.1f \
                            z:%03.1f \
                            r:%03.1f \
                            j1:%03.1f \
                            j2:%03.1f \
                            j3:%03.1f \
                            j4:%03.1f"
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
        return response

    def _read_message(self) -> Optional[Message]:
        """
        Reads a message from the serial connection.

        Returns:
            The received Message object, or None if no
                message is read.
        """
        time.sleep(0.1)
        b = self.conn.serial_conn.read_all()
        if len(b) > 0:
            msg = Message(b)
            if self.verbose:
                print("pydobot: <<", msg)
            return msg

        warnings.warn("Read null message. Please verify if something went wrong")
        return None

    def _send_command(self, msg: Message, wait: bool = False) -> Message:
        """
        Sends a message to the Dobot and optionally waits for its execution.

        Args:
            msg (Message): The message object to send.
            wait: If True, waits for the command to be executed by the
                robot (default: False).

        Returns:
            Message: The response message from the Dobot.

        Raises:
            TypeError: If no response is received from the Dobot.
        """
        self.lock.acquire()
        self._send_message(msg)
        response = self._read_message()
        self.lock.release()

        if response is None:
            raise TypeError(f"Response is none. Something went wrong. Send msg: {msg}")

        if not wait:
            return response

        expected_idx = struct.unpack_from("<L", response.params, 0)[0]
        if self.verbose:
            print("pydobot: waiting for command", expected_idx)

        while True:
            current_idx = self.get_queued_cmd_current_index()

            if current_idx != expected_idx:
                time.sleep(0.1)
                continue

            if self.verbose:
                print("pydobot: command %d executed" % current_idx)
            break

        return response

    def _send_message(self, msg: Message) -> None:
        """
        Writes a message to the Dobot's serial connection.

        Args:
            msg (Message): The message object to send.
        """
        time.sleep(0.1)
        if self.verbose:
            print("pydobot: >>", msg)
        self.conn.serial_conn.write(msg.bytes())

    def set_cp_cmd(self, cmd: tagCPCmd) -> Message:
        """
        Executes a CP (Continuous Path) command to move the end-effector to a
        specified Cartesian coordinate.

        Args:
            cmd (tagCPCmd): An object having the xyz coordinates,
                the cpMode and the velocity_or_power

        Returns:
            Message: The response message from the Dobot.
        """
        params = bytearray()
        params.extend(cmd.pack())
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_CP_CMD, ControlValues.Both, params
        )

    def set_end_effector_gripper(self, enable: bool = False) -> Message:
        """
        Sets the status of the gripper (open/close).

        Args:
            enable: True to enable (close) the gripper, False to
                disable (open) (default: False).

        Returns:
            Message: The response message from the Dobot.
        """
        params = bytearray([0x01, (0x01 if enable else 0x00)])
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_END_EFFECTOR_GRIPPER,
            ControlValues.Both,
            params,
        )

    def set_end_effector_suction_cup(self, enable: bool = False) -> Message:
        """
        Sets the status of the suction cup (on/off).

        Args:
            enable: True to turn on the suction cup, False to turn off
                (default: False).

        Returns:
            Message: The response message from the Dobot.
        """
        params = bytearray([0x01, (0x01 if enable else 0x00)])
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_END_EFFECTOR_SUCTION_CUP,
            ControlValues.Both,
            params,
        )

    def set_ptp_joint_params(self, params: tagPTPJointParams) -> Message:
        """
        Sets the velocity and acceleration ratios for each joint in PTP
        (Point-to-Point) mode.

        Args:
            params (tagPTPJointParams): An object containing the joint
                parameters.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_JOINT_PARAMS,
            ControlValues.Both,
            params.pack(),
        )

    def set_ptp_coordinate_params(self, params: tagPTPCoordinateParams) -> Message:
        """
        Sets the velocity and acceleration of the Cartesian coordinate axes
        in PTP mode.

        Args:
            params (tagPTPCoordinateParams): An object containing the
                coordinate parameters.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_COORDINATE_PARAMS,
            ControlValues.Both,
            params.pack(),
        )

    def set_ptp_jump_params(self, params: tagPTPJumpParams) -> Message:
        """
        Sets the lifting height and maximum lifting height for JUMP mode in PTP
        movements.

        Args:
            params (tagPTPJumpParams): An object containing the jump
                parameters.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_JUMP_PARAMS,
            ControlValues.Both,
            params.pack(),
        )

    def set_ptp_common_params(self, params: tagPTPCommonParams) -> Message:
        """
        Sets the common velocity and acceleration ratios for PTP mode.

        Args:
            params (tagPTPCommonParams): An object containing the common PTP
                parameters.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_COMMON_PARAMS,
            ControlValues.Both,
            params.pack(),
        )

    def set_ptp_cmd(self, cmd: tagPTPCmd, wait: bool) -> Message:
        """
        Executes a PTP (Point-to-Point) movement command to a specified
        Cartesian coordinate with a given mode.

        Args:
            cmd (tagPTPCmd): An object that contains the xyzr values and the
                mode of movement.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        params = bytearray(cmd.pack())
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_PTP_CMD, ControlValues.Both, params, wait
        )

    def set_queued_cmd_clear(self) -> Message:
        """
        Clears the command queue.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_QUEUED_CMD_CLEAR, ControlValues.ReadWrite
        )

    def set_queued_cmd_start_exec(self) -> Message:
        """
        Starts the execution of commands in the queue.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_QUEUED_CMD_START_EXEC, ControlValues.ReadWrite
        )

    def set_wait_cmd(self, params: tagWAITCmd) -> Message:
        """
        Adds a wait command to the queue.

        Args:
            params (tagWAITCmd): An object containing the wait parameters.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_WAIT_CMD, ControlValues.Both, params.pack()
        )

    def set_queued_cmd_stop_exec(self) -> Message:
        """
        Stops the execution of commands in the queue.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_QUEUED_CMD_STOP_EXEC, ControlValues.ReadWrite
        )

    def set_device_sn(self, device_serial_number: str) -> Message:
        """
        Sets the device serial number.

        Args:
            device_serial_number: The serial number to set.

        Returns:
            Message: The response message from the Dobot.
        """
        params = bytearray(device_serial_number.encode("utf-8"))
        return self._send_command_with_params(
            CommunicationProtocolIDs.GET_SET_DEVICE_SN,
            ControlValues.ReadWrite,
            params,
        )

    def get_device_sn(self) -> Message:
        """
        Gets the device serial number.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.GET_SET_DEVICE_SN, ControlValues.Zero
        )

    def set_device_name(self, device_name: str) -> Message:
        """
        Sets the device name.

        Args:
            device_name: The name to set.

        Returns:
            Message: The response message from the Dobot.
        """
        params = bytearray(device_name.encode("utf-8"))
        return self._send_command_with_params(
            CommunicationProtocolIDs.GET_SET_DEVICE_NAME,
            ControlValues.ReadWrite,
            params,
        )

    def get_device_name(self) -> Message:
        """
        Gets the device name.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.GET_SET_DEVICE_NAME, ControlValues.Zero
        )

    def get_device_version(self) -> Message:
        """
        Gets the device firmware version.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.GET_DEVICE_VERSION, ControlValues.Zero
        )

    def set_device_rail_capability(
        self, enable: bool, version: tagVersionRail
    ) -> Message:
        """
        Sets the device's rail capability.

        Args:
            enable: True to enable rail capability, False to disable.
            version (tagVersionRail): The version of the rail.

        Returns:
            Message: The response message from the Dobot.
        """
        tag = tagWithL(enable, version)
        params = tag.pack()
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_DEVICE_WITH_RAIL,
            ControlValues.ReadWrite,
            params,
        )

    def get_device_rail_capability(self) -> Message:
        """
        Gets the device's rail capability.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_DEVICE_WITH_RAIL, ControlValues.Zero
        )

    def get_device_time(self) -> Message:
        """
        Gets the device's internal time.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.GET_DEVICE_TIME, ControlValues.Zero
        )

    def get_device_id(self) -> Message:
        """
        Gets the device ID.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.GET_DEVICE_ID, ControlValues.Zero
        )

    def reset_pose(
        self, manual: int, rearArmAngle: float, frontArmAngle: float
    ) -> Message:
        """
        Resets the real-time pose of the robot.

        Args:
            manual: Manual reset flag.
            rearArmAngle: Rear arm angle for reset.
            frontArmAngle: Front arm angle for reset.

        Returns:
            Message: The response message from the Dobot.
        """
        params = bytearray([])
        params.extend(struct.pack("<Bff", manual, rearArmAngle, frontArmAngle))
        return self._send_command_with_params(
            CommunicationProtocolIDs.RESET_POSE, ControlValues.ReadWrite, params
        )

    def get_pose_rail(self) -> Message:
        """
        Gets the rail pose.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_POSE_L, ControlValues.Zero
        )
        if self.verbose:
            unpacked_response: float = struct.unpack("<f", response.params)[0]
            print(f"pydobot: l:{unpacked_response}")
        return response

    def get_alarms_state(self) -> Message:
        """
        Gets the current alarm state of the Dobot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.GET_ALARMS_STATE, ControlValues.Zero
        )

    def clear_all_alarms_state(self) -> Message:
        """
        Clears all alarm states of the Dobot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.CLEAR_ALL_ALARMS_STATE, ControlValues.Zero
        )

    def set_home_params(self, homeParams: tagHomeParams, wait: bool) -> Message:
        """
        Sets the homing parameters for the Dobot.

        Args:
            homeParams (tagHomeParams): An object containing the homing
                parameters.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_HOME_PARAMS,
            ControlValues.Both,
            homeParams.pack(),
            wait,
        )

    def get_home_params(self) -> Message:
        """
        Gets the homing parameters from the Dobot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_HOME_PARAMS, ControlValues.Zero
        )

    def set_home_cmd(self, options: tagHomeCmd, wait: bool) -> Message:
        """
        Executes the homing function.

        Args:
            options (tagHomeCmd): An object containing homing command options.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_HOME_CMD,
            ControlValues.Both,
            options.pack(),
            wait,
        )

    def set_autoleveling(self, autolevel: tagAutoLevelingParams) -> Message:
        """
        Sets automatic leveling parameters.

        Args:
            autolevel (tagAutoLevelingParams): An object containing auto-
                leveling parameters.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_AUTO_LEVELING,
            ControlValues.Both,
            autolevel.pack(),
        )

    def get_autoleveling(self) -> Message:
        """
        Gets automatic leveling parameters.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_AUTO_LEVELING, ControlValues.Zero
        )

    def set_hht_trig_mode(self, mode: HHTTrigMode) -> Message:
        """
        Sets the Hand Hold Teaching trigger mode.

        Args:
            mode (HHTTrigMode): The HHT trigger mode to set.

        Returns:
            Message: The response message from the Dobot.
        """
        params = bytearray([mode.value])
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_HHTTRIG_MODE,
            ControlValues.ReadWrite,
            params,
        )

    def get_hht_trig_mode(self) -> Message:
        """
        Gets the Hand Hold Teaching trigger mode.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_HHTTRIG_MODE, ControlValues.Zero
        )
        return response

    def set_hht_trig_output_enabled(self, is_enabled: bool) -> Message:
        """
        Enables or disables the Hand Hold Teaching trigger output.

        Args:
            is_enabled: True to enable, False to disable.

        Returns:
            Message: The response message from the Dobot.
        """
        params = bytearray([1 if is_enabled else 0])
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_HHTTRIG_OUTPUT_ENABLED,
            ControlValues.ReadWrite,
            params,
        )

    def get_hht_trig_output_enabled(self) -> Message:
        """
        Checks if the Hand Hold Teaching trigger output is enabled.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_HHTTRIG_OUTPUT_ENABLED, ControlValues.Zero
        )
        return response

    def get_hht_trig_output(self) -> Message:
        """
        Gets the current Hand Hold Teaching trigger output value.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_HHTTRIG_OUTPUT, ControlValues.Zero
        )
        return response

    def set_end_effector_params(
        self, params: tagEndEffectorParams, wait: bool
    ) -> Message:
        """
        Sets the parameters for the end effector.

        Args:
            params (tagEndEffectorParams): An object containing the end
                effector parameters.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_END_EFFECTOR_PARAMS,
            ControlValues.Both,
            params.pack(),
            wait,
        )

    def get_end_effector_params(self) -> Message:
        """
        Gets the parameters for the end effector.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_END_EFFECTOR_PARAMS, ControlValues.Zero
        )
        return response

    def set_end_effector_laser(
        self, enable_ctrl: bool, on: bool, wait: bool
    ) -> Message:
        """
        Controls the laser end effector.

        Args:
            enable_ctrl: True to enable laser control, False to disable.
            on: True to turn the laser on, False to turn it off.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        params = bytearray([enable_ctrl, on])
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_END_EFFECTOR_LASER,
            ControlValues.Both,
            params,
            wait,
        )

    def get_end_effector_laser(self) -> Message:
        """
        Gets the status of the laser end effector.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_END_EFFECTOR_LASER, ControlValues.Zero
        )
        return response

    def get_end_effector_suction_cup(self) -> Message:
        """
        Gets the status of the suction cup end effector.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_END_EFFECTOR_SUCTION_CUP,
            ControlValues.Zero,
        )
        return response

    def get_end_effector_gripper(self) -> Message:
        """
        Gets the status of the gripper end effector.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_END_EFFECTOR_GRIPPER, ControlValues.Zero
        )
        return response

    def set_jog_joint_params(self, params: tagJOGJointParams, wait: bool) -> Message:
        """
        Sets the parameters for joint mode JOG movements.

        Args:
            params (tagJOGJointParams): An object containing the JOG joint
                parameters.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_JOG_JOINT_PARAMS,
            ControlValues.Both,
            params.pack(),
            wait,
        )

    def get_jog_joint_params(self) -> Message:
        """
        Gets the parameters for joint mode JOG movements.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_JOG_JOINT_PARAMS, ControlValues.Zero
        )
        return response

    def set_jog_coordinate_params(
        self, params: tagJOGCoordinateParams, wait: bool
    ) -> Message:
        """
        Sets the parameters for coordinate mode JOG movements.

        Args:
            params (tagJOGCoordinateParams): An object containing the JOG
                coordinate parameters.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_JOG_COORDINATE_PARAMS,
            ControlValues.Both,
            params.pack(),
            wait,
        )

    def get_jog_coordinate_params(self) -> Message:
        """
        Gets the parameters for coordinate mode JOG movements.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_JOG_COORDINATE_PARAMS, ControlValues.Zero
        )
        return response

    def set_jog_common_params(self, params: tagJOGCommonParams, wait: bool) -> Message:
        """
        Sets common parameters for JOG movements.

        Args:
            params (tagJOGCommonParams): An object containing the common JOG
                parameters.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_JOG_COMMON_PARAMS,
            ControlValues.Both,
            params.pack(),
            wait,
        )

    def get_jog_common_params(self) -> Message:
        """
        Gets common parameters for JOG movements.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_JOG_COMMON_PARAMS, ControlValues.Zero
        )
        return response

    def set_jog_cmd(self, cmd: tagJOGCmd, wait: bool) -> Message:
        """
        Executes a JOG command.

        Args:
            cmd (tagJOGCmd): An object containing the JOG command.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_JOG_CMD,
            ControlValues.Both,
            cmd.pack(),
            wait,
        )

    def set_jogl_params(self, params: tagJOGLParams, wait: bool) -> Message:
        """
        Sets parameters for JOGL (Joint Jog with Linear movement) mode.

        Args:
            params (tagJOGLParams): An object containing the JOGL parameters.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_JOGL_PARAMS,
            ControlValues.ReadWrite,
            params.pack(),
            wait,
        )

    def get_jogl_params(self) -> Message:
        """
        Gets parameters for JOGL (Joint Jog with Linear movement) mode.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_JOGL_PARAMS, ControlValues.Zero
        )
        return response

    def get_ptp_joint_params(self) -> Message:
        """
        Gets the velocity and acceleration ratios for each joint in PTP
        (Point-to-Point) mode.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_JOINT_PARAMS, ControlValues.Zero
        )
        return response

    def get_ptp_coordinate_params(self) -> Message:
        """
        Gets the velocity and acceleration of the Cartesian coordinate axes
        in PTP mode.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_COORDINATE_PARAMS, ControlValues.Zero
        )
        return response

    def get_ptp_jump_params(self) -> Message:
        """
        Gets the lifting height and maximum lifting height for JUMP mode in PTP
        movements.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_JUMP_PARAMS, ControlValues.Zero
        )
        return response

    def get_ptp_common_params(self) -> Message:
        """
        Gets the common velocity and acceleration ratios for PTP mode.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_COMMON_PARAMS, ControlValues.Zero
        )
        return response

    def set_ptpl_params(self, params: tagPTPLParams, wait: bool) -> Message:
        """
        Sets parameters for PTPL (Point-to-Point with Linear movement) mode.

        Args:
            params (tagPTPLParams): An object containing the PTPL parameters.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTPL_PARAMS,
            ControlValues.Both,
            params.pack(),
            wait,
        )

    def get_ptpl_params(self) -> Message:
        """
        Gets parameters for PTPL (Point-to-Point with Linear movement) mode.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTPL_PARAMS, ControlValues.Zero
        )
        return response

    def set_ptp_with_rail_cmd(self, cmd: tagPTPWithLCmd, wait: bool) -> Message:
        """
        Executes a PTP command with rail movement.

        Args:
            cmd (tagPTPWithLCmd): An object containing the PTP command with
                rail parameters.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_PTP_WITH_L_CMD,
            ControlValues.ReadWrite,
            cmd.pack(),
            wait,
        )

    def set_ptp_jump2_params(self, params: tagPTPJump2Params, wait: bool) -> Message:
        """
        Sets the jump parameters for PTP movements with two jump heights.

        Args:
            params (tagPTPJump2Params): An object containing the PTP jump2
                parameters.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_JUMP_TO_PARAMS,
            ControlValues.ReadWrite,
            params.pack(),
            wait,
        )

    def get_ptp_jump2_params(self) -> Message:
        """
        Gets the jump parameters for PTP movements with two jump heights.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_PTP_JUMP_TO_PARAMS, ControlValues.Zero
        )
        return response

    def set_ptp_po_cmd(
        self, ptp_cmd: tagPTPCmd, po_cmds: List[tagPOCmd], wait: bool
    ) -> Message:
        """
        Executes a PTP command with multiple PO (Point Output) commands.

        Args:
            ptp_cmd (tagPTPCmd): The PTP command.
            po_cmds: A list of PO commands.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        params: bytearray = bytearray([])
        params.extend(ptp_cmd.pack())
        for po_cmd in po_cmds:
            params.extend(po_cmd.pack())
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_PTPPO_CMD, ControlValues.Both, params, wait
        )

    def set_ptp_po_with_rail_cmd(
        self, ptp_cmd: tagPTPWithLCmd, po_cmds: List[tagPOCmd], wait: bool
    ) -> Message:
        """
        Executes a PTP command with rail movement and multiple PO (Point Output)
        commands.

        Args:
            ptp_cmd (tagPTPWithLCmd): The PTP command with rail parameters.
            po_cmds: A list of PO commands.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        params: bytearray = bytearray([])
        params.extend(ptp_cmd.pack())
        for po_cmd in po_cmds:
            params.extend(po_cmd.pack())
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_PTPPO_WITH_L_CMD,
            ControlValues.Both,
            params,
            wait,
        )

    def set_cp_params(self, params: tagCPParams, wait: bool) -> Message:
        """
        Sets the parameters for CP (Continuous Path) movements.

        Args:
            params (tagCPParams): An object containing the CP parameters.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_CP_PARAMS,
            ControlValues.Both,
            params.pack(),
            wait,
        )

    def get_cp_params(self) -> Message:
        """
        Gets the parameters for CP (Continuous Path) movements.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_CP_PARAMS, ControlValues.Zero
        )
        return response

    def set_cp_le_cmd(self, cmd: tagCPCmd, wait: bool) -> Message:
        """
        Executes a CP (Continuous Path) command with linear end-effector
        movement.

        Args:
            cmd (tagCPCmd): An object containing the CP command.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_CPLE_CMD,
            ControlValues.Both,
            cmd.pack(),
            wait,
        )

    def set_arc_params(
        self,
        arcParams: tagARCParams,
        wait: bool,
    ) -> Message:
        """
        Sets the parameters for ARC (Arc) movements.

        Args:
            arcParams (tagARCParams): An object containing the ARC parameters.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_ARC_PARAMS,
            ControlValues.Both,
            arcParams.pack(),
            wait,
        )

    def get_arc_params(self) -> Message:
        """
        Gets the parameters for ARC (Arc) movements.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_ARC_PARAMS, ControlValues.Zero
        )
        return response

    def set_arc_cmd(
        self,
        cmd: tagARCCmd,
        wait: bool,
    ) -> Message:
        """
        Executes an ARC (Arc) movement command.

        Args:
            cmd (tagARCCmd): An object containing the ARC command.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_ARC_CMD,
            ControlValues.ReadWrite,
            cmd.pack(),
            wait,
        )

    def set_trig_cmd(self, cmd: tagTRIGCmd, wait: bool) -> Message:
        """
        Executes a TRIG (Trigger) command.

        Args:
            cmd (tagTRIGCmd): An object containing the TRIG command.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_TRIG_CMD,
            ControlValues.Both,
            cmd.pack(),
            wait,
        )

    def set_io_multiplexing(self, params: tagIOMultiplexing, wait: bool) -> Message:
        """
        Sets the I/O multiplexing configuration.

        Args:
            params (tagIOMultiplexing): An object containing the I/O
                multiplexing parameters.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_IO_MULTIPLEXING,
            ControlValues.Both,
            params.pack(),
            wait,
        )

    def get_io_multiplexing(self) -> Message:
        """
        Gets the I/O multiplexing configuration.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_IO_MULTIPLEXING, ControlValues.Zero
        )
        return response

    def set_io_do(self, params: tagIODO, wait: bool) -> Message:
        """
        Sets the digital output (DO) for a specific I/O.

        Args:
            params (tagIODO): An object containing the I/O DO parameters.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_IODO,
            ControlValues.Both,
            params.pack(),
            wait,
        )

    def get_io_do(self) -> Message:
        """
        Gets the digital output (DO) status for a specific I/O.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_IODO, ControlValues.Zero
        )
        return response

    def set_io_pwm(self, params: tagIOPWM, wait: bool) -> Message:
        """
        Sets the PWM (Pulse Width Modulation) output for a specific I/O.

        Args:
            params (tagIOPWM): An object containing the I/O PWM parameters.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_IO_PWM,
            ControlValues.Both,
            params.pack(),
            wait,
        )

    def get_io_pwm(self) -> Message:
        """
        Gets the PWM (Pulse Width Modulation) output status for a specific I/O.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_IO_PWM, ControlValues.Zero
        )
        return response

    def get_io_di(self) -> Message:
        """
        Gets the digital input (DI) status for a specific I/O.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_IODI, ControlValues.Zero
        )
        return response

    def get_io_adc(self) -> Message:
        """
        Gets the ADC (Analog-to-Digital Converter) value for a specific I/O.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_IO_ADC, ControlValues.Zero
        )
        return response

    def set_e_motor(self, params: tagEMOTOR, wait: bool) -> Message:
        """
        Controls an external motor.

        Args:
            params (tagEMOTOR): An object containing the external motor
                parameters.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_EMOTOR,
            ControlValues.Both,
            params.pack(),
            wait,
        )

    def set_color_sensor(self, params: tagDevice, wait: bool) -> Message:
        """
        Sets the parameters for the color sensor.

        Args:
            params (tagDevice): An object containing the device (color sensor)
                parameters.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_COLOR_SENSOR,
            ControlValues.Both,
            params.pack(),
            wait,
        )

    def get_color_sensor(self) -> Message:
        """
        Gets the readings from the color sensor.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_COLOR_SENSOR, ControlValues.Zero
        )
        return response

    def set_ir_switch(self, params: tagDevice, wait: bool) -> Message:
        """
        Sets the parameters for the IR (Infrared) switch.

        Args:
            params (tagDevice): An object containing the device (IR switch)
                parameters.
            wait: If True, waits for the command to be executed by the
                robot.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_IR_SWITCH,
            ControlValues.Both,
            params.pack(),
            wait,
        )

    def get_ir_switch(self) -> Message:
        """
        Gets the status of the IR (Infrared) switch.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_IR_SWITCH, ControlValues.Zero
        )
        return response

    def set_angle_sensor_static_error(
        self, rear_arm_angle_error: float, front_arm_angle_error: float
    ) -> Message:
        """
        Sets the static error for the angle sensors.

        Args:
            rear_arm_angle_error: The static error for the rear arm
                angle sensor.
            front_arm_angle_error: The static error for the front arm
                angle sensor.

        Returns:
            Message: The response message from the Dobot.
        """
        params = bytearray(
            struct.pack("<ff", rear_arm_angle_error, front_arm_angle_error)
        )
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_ANGLE_SENSOR_STATIC_ERROR,
            ControlValues.ReadWrite,
            params,
        )

    def get_angle_sensor_static_error(self) -> Message:
        """
        Gets the static error for the angle sensors.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_ANGLE_SENSOR_STATIC_ERROR,
            ControlValues.Zero,
        )
        return response

    def set_wifi_config_mode(self, enable: bool) -> Message:
        """
        Enables or disables Wi-Fi configuration mode.

        Args:
            enable: True to enable, False to disable.

        Returns:
            Message: The response message from the Dobot.
        """
        params = bytearray([1 if enable else 0])
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_CONFIG_MODE,
            ControlValues.ReadWrite,
            params,
        )

    def get_wifi_config_mode(self) -> Message:
        """
        Gets the Wi-Fi configuration mode status.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_CONFIG_MODE, ControlValues.Zero
        )
        return response

    def set_wifi_ssid(self, ssid: str) -> Message:
        """
        Sets the Wi-Fi SSID.

        Args:
            ssid: The Wi-Fi SSID to set.

        Returns:
            Message: The response message from the Dobot.
        """
        params = bytearray(ssid.encode("utf-8"))
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_SSID,
            ControlValues.ReadWrite,
            params,
        )

    def get_wifi_ssid(self) -> Message:
        """
        Gets the Wi-Fi SSID.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_SSID, ControlValues.Zero
        )
        return response

    def set_wifi_password(self, password: str) -> Message:
        """
        Sets the Wi-Fi password.

        Args:
            password: The Wi-Fi password to set.

        Returns:
            Message: The response message from the Dobot.
        """
        params = bytearray(password.encode("utf-8"))
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_PASSWORD,
            ControlValues.ReadWrite,
            params,
        )

    def get_wifi_password(self) -> Message:
        """
        Gets the Wi-Fi password.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_PASSWORD, ControlValues.Zero
        )
        return response

    def set_wifi_ip_address(self, params: tagWIFIIPAddress) -> Message:
        """
        Sets the Wi-Fi IP address.

        Args:
            params (tagWIFIIPAddress): An object containing the Wi-Fi IP
                address parameters.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_IP_ADDRESS,
            ControlValues.ReadWrite,
            params.pack(),
        )

    def get_wifi_ip_address(self) -> Message:
        """
        Gets the Wi-Fi IP address.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_IP_ADDRESS, ControlValues.Zero
        )
        return response

    def set_wifi_netmask(self, params: tagWIFINetmask) -> Message:
        """
        Sets the Wi-Fi netmask.

        Args:
            params (tagWIFINetmask): An object containing the Wi-Fi netmask
                parameters.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_NETMASK,
            ControlValues.ReadWrite,
            params.pack(),
        )

    def get_wifi_netmask(self) -> Message:
        """
        Gets the Wi-Fi netmask.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_NETMASK, ControlValues.Zero
        )
        return response

    def set_wifi_gateway(self, params: tagWIFIGateway) -> Message:
        """
        Sets the Wi-Fi gateway.

        Args:
            params (tagWIFIGateway): An object containing the Wi-Fi gateway
                parameters.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_GATEWAY,
            ControlValues.ReadWrite,
            params.pack(),
        )

    def get_wifi_gateway(self) -> Message:
        """
        Gets the Wi-Fi gateway.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_GATEWAY, ControlValues.Zero
        )
        return response

    def set_wifi_dns(self, params: tagWIFIDNS) -> Message:
        """
        Sets the Wi-Fi DNS server.

        Args:
            params (tagWIFIDNS): An object containing the Wi-Fi DNS parameters.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_DNS,
            ControlValues.ReadWrite,
            params.pack(),
        )

    def get_wifi_dns(self) -> Message:
        """
        Gets the Wi-Fi DNS server.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.SET_GET_WIFI_DNS, ControlValues.Zero
        )
        return response

    def get_wifi_connect_status(self) -> Message:
        """
        Gets the Wi-Fi connection status.

        Returns:
            Message: The response message from the Dobot.
        """
        response = self._send_command_with_params(
            CommunicationProtocolIDs.GET_WIFI_CONNECT_STATUS, ControlValues.Zero
        )
        return response

    def set_lost_step_params(self, value: float) -> Message:
        """
        Sets parameters related to losing-step detection.

        Args:
            value: The value for lost step parameters.

        Returns:
            Message: The response message from the Dobot.
        """
        params = bytearray(struct.pack("<f", value))
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_LOST_STEP_PARAMS,
            ControlValues.ReadWrite,
            params,
        )

    def set_lost_step_cmd(self, wait: bool = False) -> Message:
        """
        Executes a losing-step detection command.

        Args:
            wait: If True, waits for the command to be executed by the
                robot (default: False).

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_LOST_STEP_CMD, ControlValues.Both, wait=wait
        )

    def set_queued_cmd_force_stop_exec(self) -> Message:
        """
        Forces the stop of command execution in the queue.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_QUEUED_CMD_FORCE_STOP_EXEC,
            ControlValues.ReadWrite,
        )

    def set_queued_cmd_start_download(
        self, total_loop: int, line_per_loop: int
    ) -> Message:
        """
        Starts downloading commands to the queue.

        Args:
            total_loop: Total number of loops for command download.
            line_per_loop: Number of lines per loop for command
                download.

        Returns:
            Message: The response message from the Dobot.
        """
        params = bytearray(struct.pack("<II", total_loop, line_per_loop))
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_QUEUED_CMD_START_DOWNLOAD,
            ControlValues.ReadWrite,
            params,
        )

    def set_queued_cmd_stop_download(self) -> Message:
        """
        Stops downloading commands to the queue.

        Returns:
            Message: The response message from the Dobot.
        """
        return self._send_command_with_params(
            CommunicationProtocolIDs.SET_QUEUED_CMD_STOP_DOWNLOAD,
            ControlValues.ReadWrite,
        )
