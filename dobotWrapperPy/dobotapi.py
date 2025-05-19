# mypy: disable-error-code="import"
import serial

from typing import Optional, Tuple
import struct
import time
import threading
import warnings

from .message import Message
from .enums.ptpMode import PTPMode
from .enums.CommunicationProtocolIDs import CommunicationProtocolIDs
from .enums.ControlValues import ControlValues
from .enums.tagVersionRail import tagVersionRail
from .paramsStructures import tagWithL
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
        self.set_ptp_joint_params(200, 200, 200, 200, 200, 200, 200, 200)
        self.set_ptp_coordinate_params(velocity=200, acceleration=200)
        self.set_ptp_jump_params(10, 200)
        self.set_ptp_common_params(velocity=100, acceleration=100)
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
        self.x = struct.unpack_from("<f", response.params, 0)[0]
        self.y = struct.unpack_from("<f", response.params, 4)[0]
        self.z = struct.unpack_from("<f", response.params, 8)[0]
        self.r = struct.unpack_from("<f", response.params, 12)[0]
        self.j1 = struct.unpack_from("<f", response.params, 16)[0]
        self.j2 = struct.unpack_from("<f", response.params, 20)[0]
        self.j3 = struct.unpack_from("<f", response.params, 24)[0]
        self.j4 = struct.unpack_from("<f", response.params, 28)[0]

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
                % (self.x, self.y, self.z, self.r, self.j1, self.j2, self.j3, self.j4)
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

    def set_ptp_joint_params(
        self,
        v_x: float,
        v_y: float,
        v_z: float,
        v_r: float,
        a_x: float,
        a_y: float,
        a_z: float,
        a_r: float,
    ) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_JOINT_PARAMS
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack("<f", v_x)))
        msg.params.extend(bytearray(struct.pack("<f", v_y)))
        msg.params.extend(bytearray(struct.pack("<f", v_z)))
        msg.params.extend(bytearray(struct.pack("<f", v_r)))
        msg.params.extend(bytearray(struct.pack("<f", a_x)))
        msg.params.extend(bytearray(struct.pack("<f", a_y)))
        msg.params.extend(bytearray(struct.pack("<f", a_z)))
        msg.params.extend(bytearray(struct.pack("<f", a_r)))
        if self.verbose:
            print(f"pydobot: sending from set_ptp_joint_params {msg}")
        response = self._send_command(msg)
        return response

    """
        Sets the velocity and acceleration of the Cartesian coordinate axes in PTP mode
    """

    def set_ptp_coordinate_params(
        self, velocity: float, acceleration: float
    ) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_COORDINATE_PARAMS
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack("<f", velocity)))
        msg.params.extend(bytearray(struct.pack("<f", velocity)))
        msg.params.extend(bytearray(struct.pack("<f", acceleration)))
        msg.params.extend(bytearray(struct.pack("<f", acceleration)))
        if self.verbose:
            print(f"pydobot: sending from set_ptp_coordinate_params: {msg}")
        response = self._send_command(msg)
        return response

    """
       Sets the lifting height and the maximum lifting height in JUMP mode
    """

    def set_ptp_jump_params(self, jump: float, limit: float) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_JUMP_PARAMS
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack("<f", jump)))
        msg.params.extend(bytearray(struct.pack("<f", limit)))
        if self.verbose:
            print(f"pydobot: sending from set_ptp_jump_params {msg}")
        response = self._send_command(msg)
        return response

    """
        Sets the velocity ratio, acceleration ratio in PTP mode
    """

    def set_ptp_common_params(self, velocity: float, acceleration: float) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_COMMON_PARAMS
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack("<f", velocity)))
        msg.params.extend(bytearray(struct.pack("<f", acceleration)))
        if self.verbose:
            print(f"pydobot: sending from set_ptp_common_params {msg}")
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
            print(f"pydobot: sending from set_ptp_cmd {msg}")

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
            print(f"pydobot: sending from set_queued_cmd_clear {msg}")
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
            print(f"pydobot: sending from set_queued_cmd_start_exec {msg}")
        response = self._send_command(msg)
        return response

    """
        Wait command
    """

    def set_wait_cmd(self, ms: int) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_WAIT_CMD
        msg.ctrl = ControlValues.Both
        msg.params = bytearray(struct.pack("<I", ms))
        if self.verbose:
            print(f"pydobot: sending from set_wait_cmd {msg}")

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
            print(f"pydobot: sending from set_queued_cmd_stop_exec {msg}")
        response = self._send_command(msg)
        return response

    def get_eio_level(self, address: int) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_IODO
        msg.ctrl = ControlValues.Zero
        msg.params = bytearray([])
        msg.params.extend(bytearray([address]))
        if self.verbose:
            print(f"pydobot: sending from get_eio_level {msg}")

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
            print(f"pydobot: sending from set_device_sn {msg}")
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
            print(f"pydobot: sending from get_device_sn {msg}")

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
            print(f"pydobot: sending from set_device_name {msg}")

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
            print(f"pydobot: sending from get_device_name {msg}")

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
            print(f"pydobot: sending from get_device_version {msg}")

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
            print(f"pydobot: sending from set_device_rail_capability {msg}")
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
            print(f"pydobot: sending from get_device_rail_capability {msg}")
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
            print(f"pydobot: sending from get_device_time {msg}")
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
            print(f"pydobot: sending from get_device_id {msg}")
        response = self._send_command(msg)
        return response
