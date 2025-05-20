# mypy: disable-error-code="import"
import serial

from typing import Optional, Tuple, List
import struct
import time
import threading
import warnings

from .message import Message
from .enums.ptpMode import PTPMode
from .enums.CommunicationProtocolIDs import CommunicationProtocolIDs
from .enums.ControlValues import ControlValues
from .enums.tagVersionRail import tagVersionRail
from .enums.HHTTrigMode import HHTTrigMode
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
from .dobotConnection import DobotConnection


class DobotApi(threading.Thread):

    _on: bool
    verbose: bool
    lock: threading.Lock
    conn: DobotConnection
    is_open: bool
    ctrl: ControlValues

    def __init__(self, dobot_connection: DobotConnection, verbose: bool = False):
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

    def close(self) -> None:
        self._on = False
        self.lock.acquire()
        if self.verbose:
            print("pydobot: %s closed" % self.conn.serial_conn.name)
        del self.conn
        self.lock.release()

    def __del__(self) -> None:
        self.close()

    """
        Gets the current command index
    """

    def get_queued_cmd_current_index(self) -> int:
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_QUEUED_CMD_CURRENT_INDEX
        if self.verbose:
            print(f"pydobot: sending from get_queued_cmd_current_index: {msg}")
        response = self._send_command(msg)
        idx = int(struct.unpack_from("<L", response.params, 0)[0])
        return idx

    """
        Gets the real-time pose of the Dobot
    """

    def get_pose(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_POSE
        if self.verbose:
            print(f"pydobot: sending from get_pose: {msg}")
        response = self._send_command(msg)
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
        time.sleep(0.1)
        if self.verbose:
            print("pydobot: >>", msg)
        self.conn.serial_conn.write(msg.bytes())

    """
        Executes the CP Command
    """

    def set_cp_cmd(self, x: float, y: float, z: float) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_CP_CMD
        msg.ctrl = ControlValues.Both
        msg.params = bytearray(bytes([0x01]))
        msg.params.extend(bytearray(struct.pack("<f", x)))
        msg.params.extend(bytearray(struct.pack("<f", y)))
        msg.params.extend(bytearray(struct.pack("<f", z)))
        msg.params.append(0x00)
        if self.verbose:
            print(f"pydobot: sending from set_cp_cmd: {msg}")
        response = self._send_command(msg)

        return response

    """
        Sets the status of the gripper
    """

    def set_end_effector_gripper(self, enable: bool = False) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_GRIPPER
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))

        if self.verbose:
            print(f"pydobot: sending from set_end_effector_gripper: {msg}")
        response = self._send_command(msg)
        return response

    """
        Sets the status of the suction cup
    """

    def set_end_effector_suction_cup(self, enable: bool = False) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_SUCTION_CUP
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))

        if self.verbose:
            print(f"pydobot: sending from set_end_effector_suction_cup: {msg}")
        response = self._send_command(msg)
        return response

    """
        Sets the velocity ratio and the acceleration ratio in PTP mode
    """

    def set_ptp_joint_params(self, params: tagPTPJointParams) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_JOINT_PARAMS
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(params.pack())
        if self.verbose:
            print(f"pydobot: sending from set_ptp_joint_params {msg}")
        response = self._send_command(msg)
        return response

    """
        Sets the velocity and acceleration of the Cartesian coordinate axes in PTP mode
    """

    def set_ptp_coordinate_params(self, params: tagPTPCoordinateParams) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_COORDINATE_PARAMS
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(params.pack())
        if self.verbose:
            print(f"pydobot: sending from set_ptp_coordinate_params: {msg}")
        response = self._send_command(msg)
        return response

    """
       Sets the lifting height and the maximum lifting height in JUMP mode
    """

    def set_ptp_jump_params(self, params: tagPTPJumpParams) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_JUMP_PARAMS
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(params.pack())
        if self.verbose:
            print(f"pydobot: sending from set_ptp_jump_params: {msg}")
        response = self._send_command(msg)
        return response

    """
        Sets the velocity ratio, acceleration ratio in PTP mode
    """

    def set_ptp_common_params(self, params: tagPTPCommonParams) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_COMMON_PARAMS
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(params.pack())
        if self.verbose:
            print(f"pydobot: sending from set_ptp_common_params: {msg}")
        response = self._send_command(msg)
        return response

    """
        Executes PTP command
    """

    def set_ptp_cmd(
        self, x: float, y: float, z: float, r: float, mode: PTPMode, wait: bool
    ) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_PTP_CMD
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(bytearray([mode.value]))
        msg.params.extend(bytearray(struct.pack("<f", x)))
        msg.params.extend(bytearray(struct.pack("<f", y)))
        msg.params.extend(bytearray(struct.pack("<f", z)))
        msg.params.extend(bytearray(struct.pack("<f", r)))
        if self.verbose:
            print(f"pydobot: sending from set_ptp_cmd: {msg}")

        response = self._send_command(msg)
        return response

    """
        Clears command queue
    """

    def set_queued_cmd_clear(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_CLEAR
        msg.ctrl = ControlValues.ReadWrite
        if self.verbose:
            print(f"pydobot: sending from set_queued_cmd_clear: {msg}")
        response = self._send_command(msg)
        return response

    """
        Start command
    """

    def set_queued_cmd_start_exec(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_START_EXEC
        msg.ctrl = ControlValues.ReadWrite
        if self.verbose:
            print(f"pydobot: sending from set_queued_cmd_start_exec: {msg}")
        response = self._send_command(msg)
        return response

    """
        Wait command
    """

    def set_wait_cmd(self, params: tagWAITCmd) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_WAIT_CMD
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(params.pack())
        if self.verbose:
            print(f"pydobot: sending from set_wait_cmd: {msg}")

        response = self._send_command(msg)
        return response

    """
        Stop command
    """

    def set_queued_cmd_stop_exec(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_STOP_EXEC
        msg.ctrl = ControlValues.ReadWrite
        if self.verbose:
            print(f"pydobot: sending from set_queued_cmd_stop_exec: {msg}")
        response = self._send_command(msg)
        return response

    def get_eio_level(self, address: int) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_IODO
        msg.ctrl = ControlValues.Zero
        msg.params = bytearray([])
        msg.params.extend(bytearray([address]))
        if self.verbose:
            print(f"pydobot: sending from get_eio_level: {msg}")

        response = self._send_command(msg)
        return response

    def set_eio_level(self, address: int, level: int) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_IODO
        raise TypeError("Not implemented")
        # TODO: Implement EIO Service
        msg.ctrl = ControlValues.ReadWrite
        msg.params = bytearray([])
        msg.params.extend(bytearray([address]))
        msg.params.extend(bytearray([level]))
        return self._send_command(msg)

    """
        Set Device Serial Number
    """

    def set_device_sn(self, device_serial_number: str) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_SET_DEVICE_SN
        msg.ctrl = ControlValues.ReadWrite
        msg.params = bytearray([])
        msg.params.extend(device_serial_number.encode("utf-8"))
        if self.verbose:
            print(f"pydobot: sending from set_device_sn: {msg}")
        response = self._send_command(msg)
        return response

    """
        Get Device Serial Number
    """

    def get_device_sn(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_SET_DEVICE_SN
        msg.ctrl = ControlValues.Zero
        msg.params = bytearray([])
        if self.verbose:
            print(f"pydobot: sending from get_device_sn: {msg}")

        response = self._send_command(msg)
        return response

    """
        Set Device Name
    """

    def set_device_name(self, device_name: str) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_SET_DEVICE_NAME
        msg.ctrl = ControlValues.ReadWrite
        msg.params = bytearray([])
        msg.params.extend(device_name.encode("utf-8"))
        if self.verbose:
            print(f"pydobot: sending from set_device_name: {msg}")

        response = self._send_command(msg)
        return response

    """
        Get Device Name
    """

    def get_device_name(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_SET_DEVICE_NAME
        msg.ctrl = ControlValues.Zero
        if self.verbose:
            print(f"pydobot: sending from get_device_name: {msg}")

        response = self._send_command(msg)
        return response

    """
        Get Device Version
    """

    def get_device_version(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_DEVICE_VERSION
        msg.ctrl = ControlValues.Zero
        if self.verbose:
            print(f"pydobot: sending from get_device_version: {msg}")

        response = self._send_command(msg)
        return response

    """
        Set Device Capability of Rail
    """

    def set_device_rail_capability(
        self, enable: bool, version: tagVersionRail
    ) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_DEVICE_WITH_RAIL
        msg.ctrl = ControlValues.ReadWrite
        msg.params = bytearray([])
        tag = tagWithL(enable, version)
        msg.params.extend(tag.pack())
        if self.verbose:
            print(f"pydobot: sending from set_device_rail_capability: {msg}")
        response = self._send_command(msg)
        return response

    """
        Set Device Capability of Rail
    """

    def get_device_rail_capability(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_DEVICE_WITH_RAIL
        msg.ctrl = ControlValues.Zero
        msg.params = bytearray([])
        if self.verbose:
            print(f"pydobot: sending from get_device_rail_capability: {msg}")
        response = self._send_command(msg)
        return response

    """
        Get Device Time
    """

    def get_device_time(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_DEVICE_TIME
        msg.ctrl = ControlValues.Zero
        msg.params = bytearray([])
        if self.verbose:
            print(f"pydobot: sending from get_device_time: {msg}")
        response = self._send_command(msg)
        return response

    """
        Get Device ID
    """

    def get_device_id(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_DEVICE_ID
        msg.ctrl = ControlValues.Zero
        msg.params = bytearray([])
        if self.verbose:
            print(f"pydobot: sending from get_device_id: {msg}")
        response = self._send_command(msg)
        return response

    """
        Reset real time pose
    """

    def reset_pose(
        self, manual: int, rearArmAngle: float, frontArmAngle: float
    ) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.RESET_POSE
        msg.ctrl = ControlValues.ReadWrite
        msg.params = bytearray([])
        msg.params.extend(struct.pack("<B", manual))
        msg.params.extend(struct.pack("<f", rearArmAngle))
        msg.params.extend(struct.pack("<f", frontArmAngle))
        if self.verbose:
            print(f"pydobot: sending from reset_pose: {msg}")
        response = self._send_command(msg)
        return response

    """
        Get Rail Pose
    """

    def get_pose_rail(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_POSE_L
        msg.ctrl = ControlValues.Zero
        if self.verbose:
            print(f"pydobot: sending from get_pose_rail: {msg}")
        response = self._send_command(msg)
        if self.verbose:
            unpacked_response: float = struct.unpack("<f", response.params)[0]
            print(f"pydobot: l:{unpacked_response}")
        return response

    """
        Get Alarm State
    """

    def get_alarms_state(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_ALARMS_STATE
        msg.ctrl = ControlValues.Zero
        if self.verbose:
            print(f"pydobot: sending from get_alarms_state: {msg}")
        response = self._send_command(msg)
        return response

    """
        Clear All Alarms State
    """

    def clear_all_alarms_state(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.CLEAR_ALL_ALARMS_STATE
        msg.ctrl = ControlValues.Zero
        if self.verbose:
            print(f"pydobot: sending from clear_all_alarms_state: {msg}")
        response = self._send_command(msg)
        return response

    """
        Set Home Parameters
    """

    def set_home_params(self, homeParams: tagHomeParams, wait: bool) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_HOME_PARAMS
        msg.ctrl = ControlValues.Both
        # if wait:
        #     msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(homeParams.pack())
        if self.verbose:
            print(f"pydobot: sending from get_home_params: {msg}")
        response = self._send_command(msg, wait)
        return response

    """
        Get Home Parameters
    """

    def get_home_params(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_HOME_PARAMS
        msg.ctrl = ControlValues.Zero
        if self.verbose:
            print(f"pydobot: sending from get_home_params: {msg}")
        response = self._send_command(msg)
        return response

    """
        Execute Homing Function
    """

    def set_home_cmd(self, options: tagHomeCmd, wait: bool) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_HOME_CMD
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(options.pack())
        if self.verbose:
            print(f"pydobot: sending from set_home_cmd: {msg}")
        response = self._send_command(msg, wait)
        return response

    """
        Set Automatic Leveling
    """

    def set_autoleveling(self, autolevel: tagAutoLevelingParams) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_AUTO_LEVELING
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(autolevel.pack())
        if self.verbose:
            print(f"pydobot: sending from set_autoleveling: {msg}")
        response = self._send_command(msg)
        return response

    """
        Get Automatic Leveling
    """

    def get_autoleveling(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_AUTO_LEVELING
        msg.ctrl = ControlValues.Zero
        if self.verbose:
            print(f"pydobot: sending from get_autoleveling: {msg}")
        response = self._send_command(msg)
        return response

    """
        Set Hand Hold Teaching Mode
    """

    def set_hht_trig_mode(self, mode: HHTTrigMode) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_HHTTRIG_MODE
        msg.ctrl = ControlValues.ReadWrite
        msg.params = bytearray([])
        msg.params.extend(struct.pack("<B", mode.value))
        if self.verbose:
            print(f"pydobot: sending from get_autoleveling: {msg}")
        response = self._send_command(msg)
        return response

    def get_hht_trig_mode(self) -> Message:
        """
        Gets the hand-hold teaching mode.
        Document reference: 1.7.1 Set/Get HHTTrigMode
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_HHTTRIG_MODE
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_hht_trig_mode: {msg}")
        response = self._send_command(msg)
        # The response payload is HHTTrigMode (enum/uint8_t)
        # In a real implementation, you would unpack response.params here
        return response

    def set_hht_trig_output_enabled(self, is_enabled: bool) -> Message:
        """
        Sets the status of the hand-holding teaching function (output enabled).
        Document reference: 1.7.2 Set/Get HHTTrigOutputEnabled
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_HHTTRIG_OUTPUT_ENABLED
        msg.ctrl = ControlValues.ReadWrite  # rw=1, isQueued=0 -> Ctrl = 1
        # Need to pack is_enabled (uint8_t) into bytes
        msg.params = bytearray([])
        msg.params.extend(struct.pack("<B", 1 if is_enabled else 0))
        if self.verbose:
            print(f"pydobot: sending from set_hht_trig_output_enabled: {msg}")
        response = self._send_command(msg)
        return response

    def get_hht_trig_output_enabled(self) -> Message:
        """
        Gets the status of the hand-hold teaching (output enabled).
        Document reference: 1.7.2 Set/Get HHTTrigOutputEnabled
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_HHTTRIG_OUTPUT_ENABLED
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_hht_trig_output_enabled: {msg}")
        response = self._send_command(msg)
        # The response payload is uint8_t: isEnabled
        # In a real implementation, you would unpack response.params here
        return response

    def get_hht_trig_output(self) -> Message:
        """
        Gets the hand-hold teaching trigger signal.
        Document reference: 1.7.3 Get HHTTrigOutput
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_HHTTRIG_OUTPUT
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_hht_trig_output: {msg}")
        response = self._send_command(msg)
        # The response payload is uint8_t: isTriggered
        # In a real implementation, you would unpack response.params here
        return response

    # --- EndEffector ---

    def set_end_effector_params(
        self, params: tagEndEffectorParams, wait: bool
    ) -> Message:
        """
        Sets the offset parameters of the end-effector.
        Document reference: 1.8.1 Set/Get EndEffectorParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_PARAMS
        msg.ctrl = ControlValues.Both  # rw=1
        # Need to pack EndEffectorParams object into bytes
        # Assuming params object has a to_bytes method
        # msg.params = params.to_bytes() # Placeholder
        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_end_effector_params: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def get_end_effector_params(self) -> Message:
        """
        Gets the offset parameters of the end-effector.
        Document reference: 1.8.1 Set/Get EndEffectorParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_PARAMS
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_end_effector_params: {msg}")
        response = self._send_command(msg)
        # The response payload is EndEffectorParams (struct with 3 floats)
        # In a real implementation, you would unpack response.params into an EndEffectorParams object
        return response

    def set_end_effector_laser(
        self, enable_ctrl: bool, on: bool, wait: bool
    ) -> Message:
        """
        Sets the status of the laser end-effector.
        Document reference: 1.8.2 Set/Get EndEffectorLaser
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_LASER
        msg.ctrl = ControlValues.Both  # rw=1
        # Need to pack enable_ctrl (uint8_t) and on (uint8_t) into bytes
        msg.params = bytearray([])
        msg.params.extend(struct.pack("<B", enable_ctrl))
        msg.params.extend(struct.pack("<B", on))
        if self.verbose:
            print(f"pydobot: sending from set_end_effector_laser: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def get_end_effector_laser(self) -> Message:
        """
        Gets the status of the laser end-effector.
        Document reference: 1.8.2 Set/Get EndEffectorLaser
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_LASER
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_end_effector_laser: {msg}")
        response = self._send_command(msg)
        # The response payload is uint8_t: isCtrlEnabled, uint8_t: isOn
        # In a real implementation, you would unpack response.params here
        return response

    def get_end_effector_suction_cup(self) -> Message:
        """
        Gets the status of the air pump (suction cup end-effector).
        Document reference: 1.8.3 Set/Get EndEffectorSuctionCup
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_SUCTION_CUP
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_end_effector_suction_cup: {msg}")
        response = self._send_command(msg)
        # The response payload is uint8_t: isCtrlEnable, uint8_t: issucked
        # In a real implementation, you would unpack response.params here
        return response

    def get_end_effector_gripper(self) -> Message:
        """
        Gets the status of the gripper end-effector.
        Document reference: 1.8.4 Set/Get EndEffectorGripper
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_GRIPPER
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_end_effector_gripper: {msg}")
        response = self._send_command(msg)
        # The response payload is uint8_t: isCtrlEnable, uint8_t: isGripped
        # In a real implementation, you would unpack response.params here
        return response

    # --- JOG ---

    def set_jog_joint_params(self, params: tagJOGJointParams, wait: bool) -> Message:
        """
        Sets the velocity and acceleration of the joints coordinate axes in jogging mode.
        Document reference: 1.9.1 Set/Get JOGJointParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_JOG_JOINT_PARAMS
        msg.ctrl = ControlValues.Both  # rw=1
        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_jog_joint_params: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def get_jog_joint_params(self) -> Message:
        """
        Gets the velocity and acceleration of the joints coordinate axes in jogging mode.
        Document reference: 1.9.1 Set/Get JOGJointParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_JOG_JOINT_PARAMS
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_jog_joint_params: {msg}")
        response = self._send_command(msg)
        # The response payload is JOGJointParams (struct with 8 floats)
        # In a real implementation, you would unpack response.params into a JOGJointParams object
        return response

    def set_jog_coordinate_params(
        self, params: tagJOGCoordinateParams, wait: bool
    ) -> Message:
        """
        Sets the velocity and acceleration of the Cartesian coordinate axes in jogging mode.
        Document reference: 1.9.2 Set/Get JOGCoordinateParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_JOG_COORDINATE_PARAMS
        msg.ctrl = ControlValues.Both  # rw=1
        # Need to pack JOGCoordinateParams object into bytes (8 floats)
        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_jog_coordinate_params: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def get_jog_coordinate_params(self) -> Message:
        """
        Gets the velocity and acceleration of the Cartesian coordinate axes in jogging mode.
        Document reference: 1.9.2 Set/Get JOGCoordinateParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_JOG_COORDINATE_PARAMS
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_jog_coordinate_params: {msg}")
        response = self._send_command(msg)
        # The response payload is JOGCoordinateParams (struct with 8 floats)
        # In a real implementation, you would unpack response.params into a JOGCoordinateParams object
        return response

    def set_jog_common_params(self, params: tagJOGCommonParams, wait: bool) -> Message:
        """
        Sets the velocity ratio and acceleration ratio in jogging mode.
        Document reference: 1.9.3 Set/Get JOGCommonParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_JOG_COMMON_PARAMS
        msg.ctrl = ControlValues.Both  # rw=1
        # Need to pack JOGCommonParams object into bytes (2 floats)
        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_jog_common_params: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def get_jog_common_params(self) -> Message:
        """
        Gets the velocity ratio and acceleration ratio in jogging mode.
        Document reference: 1.9.3 Set/Get JOGCommonParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_JOG_COMMON_PARAMS
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_jog_common_params: {msg}")
        response = self._send_command(msg)
        # The response payload is JOGCommonParams (struct with 2 floats)
        # In a real implementation, you would unpack response.params into a JOGCommonParams object
        return response

    def set_jog_cmd(self, cmd: tagJOGCmd, wait: bool) -> Message:
        """
        Executes the jogging command.
        Document reference: 1.9.4 SetJOGCmd
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_JOG_CMD
        msg.ctrl = ControlValues.Both  # rw=1
        # Need to pack JOGCmd object into bytes (2 uint8_t)
        msg.params = bytearray([])
        msg.params.extend(cmd.pack())

        if self.verbose:
            print(f"pydobot: sending from set_jog_cmd: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def set_jogl_params(self, params: tagJOGLParams, wait: bool) -> Message:
        """
        Sets the velocity and acceleration of the sliding rail in jogging mode.
        Document reference: 1.9.5 Set/Get JOGLParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_JOGL_PARAMS
        msg.ctrl = ControlValues.ReadWrite  # rw=1

        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_jogl_params: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def get_jogl_params(self) -> Message:
        """
        Gets the velocity and acceleration of the sliding rail in jogging mode.
        Document reference: 1.9.5 Set/Get JOGLParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_JOGL_PARAMS
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_jogl_params: {msg}")
        response = self._send_command(msg)
        # The response payload is JOGLParams (struct with 2 floats)
        # In a real implementation, you would unpack response.params into a JOGLParams object
        return response

    # --- PTP ---

    def get_ptp_joint_params(self) -> Message:
        """
        Gets the velocity and acceleration of the joint coordinate axes in PTP mode.
        Document reference: 1.10.1 Set/Get PTPJointParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_JOINT_PARAMS
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_ptp_joint_params: {msg}")
        response = self._send_command(msg)
        # The response payload is PTPJointParams (struct with 8 floats)
        # In a real implementation, you would unpack response.params into a PTPJointParams object
        return response

    def get_ptp_coordinate_params(self) -> Message:
        """
        Gets the velocity and acceleration of the Cartesian coordinate axes in PTP mode.
        Document reference: 1.10.2 Set/Get PTPCoordinateParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_COORDINATE_PARAMS
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_ptp_coordinate_params: {msg}")
        response = self._send_command(msg)
        # The response payload is PTPCoordinateParams (struct with 4 floats)
        # In a real implementation, you would unpack response.params into a PTPCoordinateParams object
        return response

    def get_ptp_jump_params(self) -> Message:
        """
        Gets the lifting height and the maximum lifting height in JUMP mode.
        Document reference: 1.10.3 Set/Get PTPJumpParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_JUMP_PARAMS
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_ptp_jump_params: {msg}")
        response = self._send_command(msg)
        # The response payload is PTPJumpParams (struct with 2 floats)
        # In a real implementation, you would unpack response.params into a PTPJumpParams object
        return response

    def get_ptp_common_params(self) -> Message:
        """
        Gets the velocity ratio and the acceleration ratio in PTP mode.
        Document reference: 1.10.4 Set/Get PTPCommonParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_COMMON_PARAMS
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_ptp_common_params: {msg}")
        response = self._send_command(msg)
        # The response payload is PTPCommonParams (struct with 2 floats)
        # In a real implementation, you would unpack response.params into a PTPCommonParams object
        return response

    def set_ptpl_params(self, params: tagPTPLParams, wait: bool) -> Message:
        """
        Sets the velocity and acceleration of the sliding rail in PTP mode.
        Document reference: 1.10.6 Set/Get PTPLParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTPL_PARAMS
        msg.ctrl = ControlValues.Both  # rw=1

        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_ptp_l_params: {msg}")
        response = self._send_command(msg)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def get_ptpl_params(self) -> Message:
        """
        Gets the velocity and acceleration of the sliding rail in PTP mode.
        Document reference: 1.10.6 Set/Get PTPLParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTPL_PARAMS
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_ptp_l_params: {msg}")
        response = self._send_command(msg)
        # The response payload is PTPLParams (struct with 2 floats)
        # In a real implementation, you would unpack response.params into a PTPLParams object
        return response

    def set_ptp_with_rail_cmd(self, cmd: tagPTPWithLCmd, wait: bool) -> Message:
        """
        Executes PTP command with the sliding rail.
        Document reference: 1.10.7 SetPTPWithLCmd
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_PTP_WITH_L_CMD
        msg.ctrl = ControlValues.ReadWrite  # rw=1

        msg.params = bytearray([])
        msg.params.extend(cmd.pack())

        if self.verbose:
            print(f"pydobot: sending from set_ptp_with_rail_cmd: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def set_ptp_jump2_params(self, params: tagPTPJump2Params, wait: bool) -> Message:
        """
        Sets the extended parameters in JUMP mode (start/end jump height, z limit).
        Document reference: 1.10.8 Set/Get PTPJump2Params
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_JUMP_TO_PARAMS
        msg.ctrl = ControlValues.ReadWrite  # rw=1
        # Need to pack PTPJump2Params object into bytes (3 floats)

        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_ptp_jump2_params: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def get_ptp_jump2_params(self) -> Message:
        """
        Gets the extended parameters in JUMP mode (start/end jump height, z limit).
        Document reference: 1.10.8 Set/Get PTPJump2Params
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_JUMP_TO_PARAMS
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_ptp_jump2_params: {msg}")
        response = self._send_command(msg)
        # The response payload is PTPJump2Params (struct with 3 floats)
        # In a real implementation, you would unpack response.params into a PTPJump2Params object
        return response

    def set_ptp_po_cmd(
        self, ptp_cmd: tagPTPCmd, po_cmds: List[tagPOCmd], wait: bool
    ) -> Message:
        """
        Executes PTP command and manipulates I/O at certain moments.
        Document reference: 1.10.9 SetPTPPOCmd
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_PTPPO_CMD
        msg.ctrl = ControlValues.Both  # rw=1
        # Need to pack PTPCmd (uint8_t + 4 floats) and a list of POCmds (uint8_t, uint16_t, uint8_t each) into bytes

        msg.params = bytearray([])
        msg.params.extend(ptp_cmd.pack())

        for po_cmd in po_cmds:
            msg.params.extend(po_cmd.pack())

        if self.verbose:
            print(f"pydobot: sending from set_ptp_po_cmd: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def set_ptp_po_with_rail_cmd(
        self, ptp_cmd: tagPTPWithLCmd, po_cmds: List[tagPOCmd], wait: bool
    ) -> Message:
        """
        Executes PTP command with sliding rail and manipulates I/O at certain moments.
        Document reference: 1.10.10 SetPTPPOWithLCmd
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_PTPPO_WITH_L_CMD
        msg.ctrl = ControlValues.Both  # rw=1
        # Need to pack PTPWithLCmd (uint8_t + 5 floats) and a list of POCmds (uint8_t, uint16_t, uint8_t each) into bytes

        msg.params = bytearray([])
        msg.params.extend(ptp_cmd.pack())

        for po_cmd in po_cmds:
            msg.params.extend(po_cmd.pack())

        if self.verbose:
            print(f"pydobot: sending from set_ptp_po_with_l_cmd: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    # --- CP ---

    def set_cp_params(self, params: tagCPParams, wait: bool) -> Message:
        """
        Sets parameters of continuous trajectory (CP mode).
        Document reference: 1.11.1 Set/Get CPParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_CP_PARAMS
        msg.ctrl = ControlValues.Both  # rw=1
        # Need to pack CPParams object into bytes (3 floats + uint8_t)
        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_cp_params: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def get_cp_params(self) -> Message:
        """
        Gets the velocity and acceleration in CP mode.
        Document reference: 1.11.1 Set/Get CPParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_CP_PARAMS
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_cp_params: {msg}")
        response = self._send_command(msg)
        # The response payload is CPParams (struct with 3 floats + uint8_t)
        # In a real implementation, you would unpack response.params into a CPParams object
        return response

    def set_cp_le_cmd(self, cmd: tagCPCmd, wait: bool) -> Message:
        """
        Executes the continuous path laser engraving commands.
        Document reference: 1.11.3 SetCPLECmd
        Note: Uses the same structure as CPCmd but velocity_or_power is specifically laser power.
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_CPLE_CMD
        msg.ctrl = ControlValues.Both  # rw=1
        # Need to pack CPCmd object into bytes (uint8_t + 4 floats)

        msg.params = bytearray([])
        msg.params.extend(cmd.pack())

        if self.verbose:
            print(f"pydobot: sending from set_cp_le_cmd: {msg}")
        response = self._send_command(msg)
        # If is_queued is True, response will have queued_cmd_index
        return response

    # --- ARC ---

    def set_arc_params(
        self,
        arcParams: tagARCParams,
        wait: bool,
    ) -> Message:
        """
        Sets the velocity and acceleration in ARC mode.
        Document reference: 1.12.1 Set/Get ARCParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_ARC_PARAMS
        msg.ctrl = ControlValues.Both  # rw=1
        # Need to pack 4 floats into bytes
        msg.params = bytearray([])
        msg.params.extend(arcParams.pack())

        if self.verbose:
            print(f"pydobot: sending from set_arc_params: {msg}")
        response = self._send_command(msg)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def get_arc_params(self) -> Message:
        """
        Gets the velocity and acceleration in ARC mode.
        Document reference: 1.12.1 Set/Get ARCParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_ARC_PARAMS
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_arc_params: {msg}")
        response = self._send_command(msg)
        # The response payload is ARCParams (struct with 4 floats)
        # In a real implementation, you would unpack response.params here
        return response

    def set_arc_cmd(
        self,
        cmd: tagARCCmd,
        wait: bool,
    ) -> Message:
        """
        Executes the ARC command.
        Document reference: 1.12.2 SetARCCmd
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_ARC_CMD
        msg.ctrl = ControlValues.ReadWrite  # rw=1
        # Need to pack 8 floats into bytes (cirPoint and toPoint structs)

        msg.params = bytearray([])
        msg.params.extend(cmd.pack())

        if self.verbose:
            print(f"pydobot: sending from set_arc_cmd: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    # --- WAIT ---

    # --- TRIG ---

    def set_trig_cmd(self, cmd: tagTRIGCmd, wait: bool) -> Message:
        """
        Executes the triggering command.
        Document reference: 1.14.1 SetTRIGCmd
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_TRIG_CMD
        msg.ctrl = ControlValues.Both  # rw=1
        # Need to pack TRIGCmd object into bytes (uint8_t, uint8_t, uint8_t, uint16_t)

        msg.params = bytearray([])
        msg.params.extend(cmd.pack())

        if self.verbose:
            print(f"pydobot: sending from set_trig_cmd: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    # --- EIO ---

    def set_io_multiplexing(self, params: tagIOMultiplexing, wait: bool) -> Message:
        """
        Sets the I/O multiplexing.
        Document reference: 1.15.1 Set/Get IOMultiplexing
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_IO_MULTIPLEXING
        msg.ctrl = ControlValues.Both  # rw=1
        # Need to pack IOMultiplexing object into bytes (uint8_t, uint8_t)
        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_io_multiplexing: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def get_io_multiplexing(self) -> Message:
        """
        Gets the I/O multiplexing settings.
        Document reference: 1.15.1 Set/Get IOMultiplexing
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_IO_MULTIPLEXING
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_io_multiplexing: {msg}")
        response = self._send_command(msg)
        # The response payload is IOMultiplexing (struct with uint8_t, uint8_t)
        # In a real implementation, you would unpack response.params here
        return response

    def set_io_do(self, params: tagIODO, wait: bool) -> Message:
        """
        Sets the I/O digital output.
        Document reference: 1.15.2 Set/Get IODO
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_IODO
        msg.ctrl = ControlValues.Both  # rw=1

        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_io_do: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def get_io_do(self) -> Message:
        """
        Gets the I/O digital output status.
        Document reference: 1.15.2 Set/Get IODO
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_IODO
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_io_do: {msg}")
        response = self._send_command(msg)
        # The response payload is IODO (struct with uint8_t, uint8_t)
        # In a real implementation, you would unpack response.params here
        return response

    def set_io_pwm(self, params: tagIOPWM, wait: bool) -> Message:
        """
        Sets the I/O PWM output.
        Document reference: 1.15.3 Set/Get IOPWM
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_IO_PWM
        msg.ctrl = ControlValues.Both  # rw=1
        # Need to pack IOPWM object into bytes (uint8_t, float, float)

        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_io_pwm: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def get_io_pwm(self) -> Message:
        """
        Gets the I/O PWM settings.
        Document reference: 1.15.3 Set/Get IOPWM
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_IO_PWM
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_io_pwm: {msg}")
        response = self._send_command(msg)
        # The response payload is IOPWM (struct with uint8_t, float, float)
        # In a real implementation, you would unpack response.params here
        return response

    def get_io_di(self) -> Message:
        """
        Gets the I/O digital input status.
        Document reference: 1.15.4 Get IODI
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_IODI
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_io_di: {msg}")
        response = self._send_command(msg)
        # The response payload is IODI (struct with uint8_t, uint8_t)
        # In a real implementation, you would unpack response.params here
        return response

    def get_io_adc(self) -> Message:
        """
        Gets the A/D input value.
        Document reference: 1.15.5 GetIOADC
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_IO_ADC
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_io_adc: {msg}")
        response = self._send_command(msg)
        # The response payload is IOADC (struct with uint8_t, uint16_t)
        # In a real implementation, you would unpack response.params here
        return response

    def set_e_motor(self, params: tagEMOTOR, wait: bool) -> Message:
        """
        Sets the velocity of the extended motor.
        Document reference: 1.15.6 Set EMotor
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_EMOTOR
        msg.ctrl = ControlValues.Both  # rw=1
        # Need to pack EMotor object into bytes (uint8_t, uint8_t, float)
        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_e_motor: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def set_color_sensor(self, params: tagDevice, wait: bool) -> Message:
        """
        Enables the color sensor.
        Document reference: 1.15.7 Set/Get ColorSensor
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_COLOR_SENSOR
        msg.ctrl = ControlValues.Both  # rw=1
        # Need to pack DeviceColorSense object into bytes (uint8_t, Port, Version)
        # Assuming Port is uint8_t for packing purposes
        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_color_sensor: {msg}")
        response = self._send_command(msg)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def get_color_sensor(self) -> Message:
        """
        Gets the color sensor value.
        Document reference: 1.15.7 Set/Get ColorSensor
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_COLOR_SENSOR
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_color_sensor: {msg}")
        response = self._send_command(msg)
        # The response payload is Color (struct with 3 uint8_t)
        # In a real implementation, you would unpack response.params here
        return response

    def set_ir_switch(self, params: tagDevice, wait: bool) -> Message:
        """
        Sets the infrared sensor.
        Document reference: 1.15.8 Set/Get IRSwitch
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_IR_SWITCH
        msg.ctrl = ControlValues.Both  # rw=1
        # Need to pack DeviceIRSense object into bytes (uint8_t, Port, Version)
        # Assuming Port is uint8_t for packing purposes
        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_ir_switch: {msg}")
        response = self._send_command(msg)
        # If is_queued is True, response will have queued_cmd_index
        return response

    def get_ir_switch(self) -> Message:
        """
        Gets the infrared sensor state.
        Document reference: 1.15.8 Set/Get IRSwitch
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_IR_SWITCH
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_ir_switch: {msg}")
        response = self._send_command(msg)
        # The response payload is Uint8_t state
        # In a real implementation, you would unpack response.params here
        return response

    # --- Calibration (CAL) ---

    def set_angle_sensor_static_error(
        self, rear_arm_angle_error: float, front_arm_angle_error: float
    ) -> Message:
        """
        Sets the angle sensor static error.
        Document reference: 1.16.1 Set/Get AngleSensorStaticError
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_ANGLE_SENSOR_STATIC_ERROR
        msg.ctrl = ControlValues.ReadWrite  # rw=1, isQueued=0 -> Ctrl = 1
        # Need to pack 2 floats into bytes
        msg.params = bytearray([])

        msg.params.extend(
            struct.pack("<ff", rear_arm_angle_error, front_arm_angle_error)
        )

        if self.verbose:
            print(f"pydobot: sending from set_angle_sensor_static_error: {msg}")
        response = self._send_command(msg)
        return response

    def get_angle_sensor_static_error(self) -> Message:
        """
        Gets the angle sensor static error.
        Document reference: 1.16.1 Set/Get AngleSensorStaticError
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_ANGLE_SENSOR_STATIC_ERROR
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_angle_sensor_static_error: {msg}")
        response = self._send_command(msg)
        # The response payload is float: rearArmAngleError, float: frontArmAngleError
        # In a real implementation, you would unpack response.params here
        return response

    # --- WIFI ---

    def set_wifi_config_mode(self, enable: bool) -> Message:
        """
        Enables WIFI.
        Document reference: 1.17.1 Set/Get WIFIConfigMode
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_WIFI_CONFIG_MODE
        msg.ctrl = ControlValues.ReadWrite  # rw=1, isQueued=0 -> Ctrl = 1
        # Need to pack enable (uint8_t) into bytes
        msg.params = bytearray([])
        msg.params.extend(struct.pack("<B", 1 if enable else 0))
        if self.verbose:
            print(f"pydobot: sending from set_wifi_config_mode: {msg}")
        response = self._send_command(msg)
        return response

    def get_wifi_config_mode(self) -> Message:
        """
        Gets WIFI status (enabled/disabled).
        Document reference: 1.17.1 Set/Get WIFIConfigMode
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_WIFI_CONFIG_MODE
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_wifi_config_mode: {msg}")
        response = self._send_command(msg)
        # The response payload is uint8_t: enable
        # In a real implementation, you would unpack response.params here
        return response

    def set_wifi_ssid(self, ssid: str) -> Message:
        """
        Sets the WIFI SSID.
        Document reference: 1.17.2 Set/Get WIFISSID
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_WIFI_SSID
        msg.ctrl = ControlValues.ReadWrite  # rw=1, isQueued=0 -> Ctrl = 1
        # Need to pack ssid string into bytes
        msg.params = bytearray([])
        msg.params.extend(ssid.encode("utf-8"))
        if self.verbose:
            print(f"pydobot: sending from set_wifi_ssid: {msg}")
        response = self._send_command(msg)
        return response

    def get_wifi_ssid(self) -> Message:
        """
        Gets the WIFI SSID.
        Document reference: 1.17.2 Set/Get WIFISSID
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_WIFI_SSID
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_wifi_ssid: {msg}")
        response = self._send_command(msg)
        # The response payload is char[n] ssid
        # In a real implementation, you would unpack response.params here
        return response

    def set_wifi_password(self, password: str) -> Message:
        """
        Sets the WIFI network password.
        Document reference: 1.17.3 Set/Get WIFIPassword
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_WIFI_PASSWORD
        msg.ctrl = ControlValues.ReadWrite  # rw=1, isQueued=0 -> Ctrl = 1
        # Need to pack password string into bytes
        msg.params = bytearray([])
        msg.params.extend(password.encode("utf-8"))
        if self.verbose:
            print(f"pydobot: sending from set_wifi_password: {msg}")
        response = self._send_command(msg)
        return response

    def get_wifi_password(self) -> Message:
        """
        Gets the WIFI network password.
        Document reference: 1.17.3 Set/Get WIFIPassword
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_WIFI_PASSWORD
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_wifi_password: {msg}")
        response = self._send_command(msg)
        # The response payload is char[n] password
        # In a real implementation, you would unpack response.params here
        return response

    def set_wifi_ip_address(self, params: tagWIFIIPAddress) -> Message:
        """
        Sets the WIFI IP address settings.
        Document reference: 1.17.4 Set/Get WIFIIPAddress
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_WIFI_IP_ADDRESS
        msg.ctrl = ControlValues.ReadWrite  # rw=1, isQueued=0 -> Ctrl = 1
        # Need to pack WIFIIPAddress object into bytes (uint8_t + 4 uint8_t)

        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_wifi_ip_address: {msg}")
        response = self._send_command(msg)
        return response

    def get_wifi_ip_address(self) -> Message:
        """
        Gets the WIFI IP address settings.
        Document reference: 1.17.4 Set/Get WIFIIPAddress
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_WIFI_IP_ADDRESS
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_wifi_ip_address: {msg}")
        response = self._send_command(msg)
        # The response payload is WIFIIPAddress (struct with uint8_t + 4 uint8_t)
        # In a real implementation, you would unpack response.params here
        return response

    def set_wifi_netmask(self, params: tagWIFINetmask) -> Message:
        """
        Sets the WIFI netmask.
        Document reference: 1.17.5 Set/Get WIFINetmask
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_WIFI_NETMASK
        msg.ctrl = ControlValues.ReadWrite  # rw=1, isQueued=0 -> Ctrl = 1
        # Need to pack WIFINetmask object into bytes (4 uint8_t)

        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_wifi_netmask: {msg}")
        response = self._send_command(msg)
        return response

    def get_wifi_netmask(self) -> Message:
        """
        Gets the WIFI netmask.
        Document reference: 1.17.5 Set/Get WIFINetmask
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_WIFI_NETMASK
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_wifi_netmask: {msg}")
        response = self._send_command(msg)
        # The response payload is WIFINetmask (struct with 4 uint8_t)
        # In a real implementation, you would unpack response.params here
        return response

    def set_wifi_gateway(self, params: tagWIFIGateway) -> Message:
        """
        Sets the WIFI gateway.
        Document reference: 1.17.6 Set/Get WIFIGateway
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_WIFI_GATEWAY
        msg.ctrl = ControlValues.ReadWrite  # rw=1, isQueued=0 -> Ctrl = 1
        # Need to pack WIFIGateway object into bytes (4 uint8_t)

        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_wifi_gateway: {msg}")
        response = self._send_command(msg)
        return response

    def get_wifi_gateway(self) -> Message:
        """
        Gets the WIFI gateway.
        Document reference: 1.17.6 Set/Get WIFIGateway
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_WIFI_GATEWAY
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_wifi_gateway: {msg}")
        response = self._send_command(msg)
        # The response payload is WIFIGateway (struct with 4 uint8_t)
        # In a real implementation, you would unpack response.params here
        return response

    def set_wifi_dns(self, params: tagWIFIDNS) -> Message:
        """
        Sets the WIFI DNS.
        Document reference: 1.17.7 Set/Get WIFIDNS
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_WIFI_DNS
        msg.ctrl = ControlValues.ReadWrite  # rw=1, isQueued=0 -> Ctrl = 1
        # Need to pack WIFIDNS object into bytes (4 uint8_t)

        msg.params = bytearray([])
        msg.params.extend(params.pack())

        if self.verbose:
            print(f"pydobot: sending from set_wifi_dns: {msg}")
        response = self._send_command(msg)
        return response

    def get_wifi_dns(self) -> Message:
        """
        Gets the WIFI DNS.
        Document reference: 1.17.7 Set/Get WIFIDNS
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_WIFI_DNS
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_wifi_dns: {msg}")
        response = self._send_command(msg)
        # The response payload is WIFIDNS (struct with 4 uint8_t)
        # In a real implementation, you would unpack response.params here
        return response

    def get_wifi_connect_status(self) -> Message:
        """
        Gets WIFI connection status.
        Document reference: 1.17.8 GetWIFIConnectStatus
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_WIFI_CONNECT_STATUS
        msg.ctrl = ControlValues.Zero  # rw=0, isQueued=0 -> Ctrl = 0
        if self.verbose:
            print(f"pydobot: sending from get_wifi_connect_status: {msg}")
        response = self._send_command(msg)
        # The response payload is uint8_t: isConnected
        # In a real implementation, you would unpack response.params here
        return response

    # --- Losing-Step Detection ---

    def set_lost_step_params(self, value: float) -> Message:
        """
        Sets losing-step threshold.
        Document reference: 1.18.1 SetLostStepParams
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_LOST_STEP_PARAMS
        msg.ctrl = ControlValues.ReadWrite  # rw=1, isQueued=0 -> Ctrl = 1
        # Need to pack float into bytes
        msg.params = bytearray([])
        msg.params.extend(struct.pack("<f", value))

        if self.verbose:
            print(f"pydobot: sending from set_lost_step_params: {msg}")
        response = self._send_command(msg)
        return response

    def set_lost_step_cmd(self, wait: bool = False) -> Message:
        """
        Executes losing-step detection.
        Document reference: 1.18.2 SetLostStepCmd
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_LOST_STEP_CMD
        msg.ctrl = ControlValues.Both  # rw=1
        # No parameters for this command
        msg.params = bytearray([])

        if self.verbose:
            print(f"pydobot: sending from set_lost_step_cmd: {msg}")
        response = self._send_command(msg, wait)
        # If is_queued is True, response will have queued_cmd_index
        return response

    # --- Queued execution control commands ---

    def set_queued_cmd_force_stop_exec(self) -> Message:
        """
        Forces stop of execution of commands in the queue.
        Document reference: 1.19.3 Set QueuedCmdForceStopExec
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_FORCE_STOP_EXEC
        msg.ctrl = ControlValues.ReadWrite  # rw=1, isQueued=0 -> Ctrl = 1
        # No parameters for this command
        msg.params = bytearray([])

        if self.verbose:
            print(f"pydobot: sending from set_queued_cmd_force_stop_exec: {msg}")
        response = self._send_command(msg)
        return response

    def set_queued_cmd_start_download(
        self, total_loop: int, line_per_loop: int
    ) -> Message:
        """
        Starts downloading commands for offline execution.
        Document reference: 1.19.4 Set QueuedCmdStartDownload
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_START_DOWNLOAD
        msg.ctrl = ControlValues.ReadWrite  # rw=1, isQueued=0 -> Ctrl = 1
        # Need to pack uint32_t totalLoop and uint32_t linePerLoop into bytes
        msg.params = bytearray([])

        msg.params.extend(struct.pack("<II", total_loop, line_per_loop))

        if self.verbose:
            print(f"pydobot: sending from set_queued_cmd_start_download: {msg}")
        response = self._send_command(msg)
        return response

    def set_queued_cmd_stop_download(self) -> Message:
        """
        Stops downloading commands for offline execution.
        Document reference: 1.19.5 Set QueuedCmdStopDownload
        """
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_STOP_DOWNLOAD
        msg.ctrl = ControlValues.ReadWrite  # rw=1, isQueued=0 -> Ctrl = 1
        # No parameters for this command
        msg.params = bytearray([])

        if self.verbose:
            print(f"pydobot: sending from set_queued_cmd_stop_download: {msg}")
        response = self._send_command(msg)
        return response
