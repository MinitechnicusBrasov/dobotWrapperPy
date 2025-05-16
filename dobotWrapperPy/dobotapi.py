# mypy: disable-error-code="import"
import serial

from typing import Optional, Tuple
import struct
import time
import threading

from .message import Message
from .enums.ptpMode import PTPMode
from .enums.CommunicationProtocolIDs import CommunicationProtocolIDs
from .enums.ControlValues import ControlValues


class DobotApi(threading.Thread):

    _on: bool
    verbose: bool
    lock: threading.Lock
    ser: serial.Serial
    is_open: bool
    ctrl: ControlValues

    def __init__(self, port: str, verbose: bool = False):
        threading.Thread.__init__(self)

        self._on = True
        self.verbose = verbose
        self.lock = threading.Lock()
        self.ser = serial.Serial(
            port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )
        is_open = self.ser.isOpen()
        if self.verbose:
            print(
                "pydobot: %s open" % self.ser.name
                if is_open
                else "failed to open serial port"
            )

        self._set_queued_cmd_start_exec()
        self._set_queued_cmd_clear()
        self._set_ptp_joint_params(200, 200, 200, 200, 200, 200, 200, 200)
        self._set_ptp_coordinate_params(velocity=200, acceleration=200)
        self._set_ptp_jump_params(10, 200)
        self._set_ptp_common_params(velocity=100, acceleration=100)
        self._get_pose()

    def close(self) -> None:
        self._on = False
        self.lock.acquire()
        self.ser.close()
        if self.verbose:
            print("pydobot: %s closed" % self.ser.name)
        self.lock.release()

    def __del__(self) -> None:
        self.close()

    """
        Gets the current command index
    """

    def _get_queued_cmd_current_index(self) -> int:
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_QUEUED_CMD_CURRENT_INDEX
        response = self._send_command(msg)
        if response is None:
            raise TypeError("Response is none but shouldn't be")
        idx = int(struct.unpack_from("L", response.params, 0)[0])
        return idx

    """
        Gets the real-time pose of the Dobot
    """

    def _get_pose(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_POSE
        response = self._send_command(msg)
        if response is None:
            raise TypeError("Response is none but shouldn't be")
        self.x = struct.unpack_from("f", response.params, 0)[0]
        self.y = struct.unpack_from("f", response.params, 4)[0]
        self.z = struct.unpack_from("f", response.params, 8)[0]
        self.r = struct.unpack_from("f", response.params, 12)[0]
        self.j1 = struct.unpack_from("f", response.params, 16)[0]
        self.j2 = struct.unpack_from("f", response.params, 20)[0]
        self.j3 = struct.unpack_from("f", response.params, 24)[0]
        self.j4 = struct.unpack_from("f", response.params, 28)[0]

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
        b = self.ser.read_all()
        if len(b) > 0:
            msg = Message(b)
            if self.verbose:
                print("pydobot: <<", msg)
            return msg
        return None

    def _send_command(self, msg: Message, wait: bool = False) -> Optional[Message]:
        self.lock.acquire()
        self._send_message(msg)
        response = self._read_message()
        self.lock.release()

        if not wait or response is None:
            return response

        expected_idx = struct.unpack_from("L", response.params, 0)[0]
        if self.verbose:
            print("pydobot: waiting for command", expected_idx)

        while True:
            current_idx = self._get_queued_cmd_current_index()

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
        self.ser.write(msg.bytes())

    """
        Executes the CP Command
    """

    def _set_cp_cmd(self, x: float, y: float, z: float) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_CP_CMD
        msg.ctrl = ControlValues.Both
        msg.params = bytearray(bytes([0x01]))
        msg.params.extend(bytearray(struct.pack("f", x)))
        msg.params.extend(bytearray(struct.pack("f", y)))
        msg.params.extend(bytearray(struct.pack("f", z)))
        msg.params.append(0x00)
        response = self._send_command(msg)
        if response is None:
            raise TypeError("Response is none but shouldn't be")

        return response

    """
        Sets the status of the gripper
    """

    def _set_end_effector_gripper(self, enable: bool = False) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_GRIPPER
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        response = self._send_command(msg)
        if response is None:
            raise TypeError("Response is none but shouldn't be")
        return response

    """
        Sets the status of the suction cup
    """

    def _set_end_effector_suction_cup(self, enable: bool = False) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_SUCTION_CUP
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        response = self._send_command(msg)
        if response is None:
            raise TypeError("Response is none but shouldn't be")
        return response

    """
        Sets the velocity ratio and the acceleration ratio in PTP mode
    """

    def _set_ptp_joint_params(
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
        msg.params.extend(bytearray(struct.pack("f", v_x)))
        msg.params.extend(bytearray(struct.pack("f", v_y)))
        msg.params.extend(bytearray(struct.pack("f", v_z)))
        msg.params.extend(bytearray(struct.pack("f", v_r)))
        msg.params.extend(bytearray(struct.pack("f", a_x)))
        msg.params.extend(bytearray(struct.pack("f", a_y)))
        msg.params.extend(bytearray(struct.pack("f", a_z)))
        msg.params.extend(bytearray(struct.pack("f", a_r)))
        response = self._send_command(msg)
        if response is None:
            raise TypeError("Response is none but shouldn't be")
        return response

    """
        Sets the velocity and acceleration of the Cartesian coordinate axes in PTP mode
    """

    def _set_ptp_coordinate_params(
        self, velocity: float, acceleration: float
    ) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_COORDINATE_PARAMS
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack("f", velocity)))
        msg.params.extend(bytearray(struct.pack("f", velocity)))
        msg.params.extend(bytearray(struct.pack("f", acceleration)))
        msg.params.extend(bytearray(struct.pack("f", acceleration)))
        response = self._send_command(msg)
        if response is None:
            raise TypeError("Response is none but shouldn't be")
        return response

    """
       Sets the lifting height and the maximum lifting height in JUMP mode
    """

    def _set_ptp_jump_params(self, jump: float, limit: float) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_JUMP_PARAMS
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack("f", jump)))
        msg.params.extend(bytearray(struct.pack("f", limit)))
        response = self._send_command(msg)
        if response is None:
            raise TypeError("Response is none but shouldn't be")
        return response

    """
        Sets the velocity ratio, acceleration ratio in PTP mode
    """

    def _set_ptp_common_params(self, velocity: float, acceleration: float) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_COMMON_PARAMS
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack("f", velocity)))
        msg.params.extend(bytearray(struct.pack("f", acceleration)))
        response = self._send_command(msg)
        if response is None:
            raise TypeError("Response is none but shouldn't be")
        return response

    """
        Executes PTP command
    """

    def _set_ptp_cmd(
        self, x: float, y: float, z: float, r: float, mode: PTPMode, wait: bool
    ) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_PTP_CMD
        msg.ctrl = ControlValues.Both
        msg.params = bytearray([])
        msg.params.extend(bytearray([mode.value]))
        msg.params.extend(bytearray(struct.pack("f", x)))
        msg.params.extend(bytearray(struct.pack("f", y)))
        msg.params.extend(bytearray(struct.pack("f", z)))
        msg.params.extend(bytearray(struct.pack("f", r)))
        response = self._send_command(msg)
        if response is None:
            raise TypeError("Response is none but shouldn't be")
        return response

    """
        Clears command queue
    """

    def _set_queued_cmd_clear(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_CLEAR
        msg.ctrl = ControlValues.ReadWrite
        response = self._send_command(msg)
        if response is None:
            raise TypeError("Response is none but shouldn't be")
        return response

    """
        Start command
    """

    def _set_queued_cmd_start_exec(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_START_EXEC
        msg.ctrl = ControlValues.ReadWrite
        response = self._send_command(msg)
        if response is None:
            raise TypeError("Response is none but shouldn't be")
        return response

    """
        Wait command
    """

    def _set_wait_cmd(self, ms: int) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_WAIT_CMD
        msg.ctrl = ControlValues.Both
        msg.params = bytearray(struct.pack("I", ms))
        response = self._send_command(msg)
        if response is None:
            raise TypeError("Response is none but shouldn't be")
        return response

    """
        Stop command
    """

    def _set_queued_cmd_stop_exec(self) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_STOP_EXEC
        msg.ctrl = ControlValues.ReadWrite
        response = self._send_command(msg)
        if response is None:
            raise TypeError("Response is none but shouldn't be")
        return response

    def _get_eio_level(self, address: int) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_EIO
        msg.ctrl = ControlValues.Zero
        msg.params = bytearray([])
        msg.params.extend(bytearray([address]))
        response = self._send_command(msg)
        if response is None:
            raise TypeError("Response is none but shouldn't be")
        return response

    def _set_eio_level(self, address: int, level: int) -> Message:
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_EIO
        raise TypeError("Not implemented")
        # TODO: Implement EIO Service
        msg.ctrl = ControlValues.ReadWrite
        msg.params = bytearray([])
        msg.params.extend(bytearray([address]))
        msg.params.extend(bytearray([level]))
        return self._send_command(msg)

