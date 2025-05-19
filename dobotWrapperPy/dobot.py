from .dobotapi import DobotApi
from .dobotConnection import DobotConnection
import warnings
import struct
from .enums.ptpMode import PTPMode
from typing import Tuple


class Dobot:
    dobotApiInterface: DobotApi

    def __init__(self, port: str, verbose: bool = False):
        conn = DobotConnection(port=port)
        self.dobotApiInterface = DobotApi(conn, verbose)

    def __del__(self) -> None:
        del self.dobotApiInterface

    def go(self, x: float, y: float, z: float, r: float = 0.0) -> None:
        warnings.warn("go() is deprecated, use move_to() instead")
        self.move_to(x, y, z, r)

    def move_to(
        self, x: float, y: float, z: float, r: float, wait: bool = False
    ) -> None:
        self.dobotApiInterface.set_ptp_cmd(x, y, z, r, mode=PTPMode.MOVL_XYZ, wait=wait)

    def suck(self, enable: bool) -> None:
        self.dobotApiInterface.set_end_effector_suction_cup(enable)

    def grip(self, enable: bool) -> None:
        self.dobotApiInterface.set_end_effector_gripper(enable)

    def speed(self, velocity: float = 100.0, acceleration: float = 100.0) -> None:
        self.dobotApiInterface.set_ptp_common_params(velocity, acceleration)
        self.dobotApiInterface.set_ptp_coordinate_params(velocity, acceleration)

    def wait(self, ms: int) -> None:
        self.dobotApiInterface.set_wait_cmd(ms)

    def pose(self) -> Tuple[float, float, float, float, float, float, float, float]:
        response = self.dobotApiInterface.get_pose()
        x = struct.unpack_from("f", response.params, 0)[0]
        y = struct.unpack_from("f", response.params, 4)[0]
        z = struct.unpack_from("f", response.params, 8)[0]
        r = struct.unpack_from("f", response.params, 12)[0]
        j1 = struct.unpack_from("f", response.params, 16)[0]
        j2 = struct.unpack_from("f", response.params, 20)[0]
        j3 = struct.unpack_from("f", response.params, 24)[0]
        j4 = struct.unpack_from("f", response.params, 28)[0]
        return x, y, z, r, j1, j2, j3, j4

    # TODO: Implement eio

    # def get_eio(self, addr):
    #     return self._get_eio_level(addr)
    #
    # def set_eio(self, addr, val):
    #     return self._set_eio_level(addr, val)
