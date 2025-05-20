from .dobotapi import DobotApi
from .dobotConnection import DobotConnection
import warnings
import struct
from .enums.ptpMode import PTPMode
from .message import Message
from .paramsStructures import (
    tagPTPCommonParams,
    tagPTPCoordinateParams,
    tagWAITCmd,
    tagPose,
)
from typing import Tuple


class Dobot:
    dobotApiInterface: DobotApi

    def __init__(self, port: str, verbose: bool = False) -> None:
        conn = DobotConnection(port=port)
        self.dobotApiInterface = DobotApi(conn, verbose)

    def __del__(self) -> None:
        if hasattr(self, "dobotApiInterface") and self.dobotApiInterface is not None:
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
        self.dobotApiInterface.set_ptp_common_params(
            tagPTPCommonParams(velocity, acceleration)
        )
        self.dobotApiInterface.set_ptp_coordinate_params(
            tagPTPCoordinateParams(velocity, velocity, acceleration, acceleration)
        )

    def wait(self, ms: int) -> None:
        self.dobotApiInterface.set_wait_cmd(tagWAITCmd(ms))

    def pose(self) -> Tuple[float, float, float, float, float, float, float, float]:
        response: Message = self.dobotApiInterface.get_pose()
        pos = tagPose.unpack(response.bytes())
        return (
            pos.x,
            pos.y,
            pos.z,
            pos.r,
            pos.jointAngle[0],
            pos.jointAngle[1],
            pos.jointAngle[2],
            pos.jointAngle[3],
        )

    # TODO: Implement eio
