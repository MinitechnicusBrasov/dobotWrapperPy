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
    tagPTPCmd,
)
import asyncio
from typing import Tuple


class DobotAsync:
    dobotApiInterface: DobotApi
    _loop: asyncio.AbstractEventLoop

    def __init__(self, port: str, verbose: bool = False) -> None:
        conn = DobotConnection(port=port)
        self.dobotApiInterface = DobotApi(conn, verbose)
        self._loop = asyncio.get_running_loop()

    def __del__(self) -> None:
        if hasattr(self, "dobotApiInterface") and self.dobotApiInterface is not None:
            del self.dobotApiInterface

    async def move_to_linear(self, x: float, y: float, z: float, r: float) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_ptp_cmd,
            tagPTPCmd(PTPMode.MOVL_XYZ, x, y, z, r),
            True,
            True,
        )

    async def jump(self, x: float, y: float, z: float, r: float) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_ptp_cmd,
            tagPTPCmd(PTPMode.JUMP_MOVL_XYZ, x, y, z, r),
            True,
            True,
        )

    async def clear_alarms(self) -> None:
        await self._loop.run_in_executor(
            None, self.dobotApiInterface.clear_all_alarms_state
        )

    async def suck(self, enable: bool) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_end_effector_suction_cup,
            enable,
            True,
            True,
        )

    async def grip(self, enable: bool) -> None:
        await self._loop.run_in_executor(
            None, self.dobotApiInterface.set_end_effector_gripper, enable, True, True
        )
        self.dobotApiInterface.set_end_effector_gripper(enable)

    async def speed(self, velocity: float = 100.0, acceleration: float = 100.0) -> None:
        task_1 = self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_ptp_common_params,
            tagPTPCommonParams(velocity, acceleration),
            True,
            True,
        )
        task_2 = self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_ptp_coordinate_params,
            tagPTPCoordinateParams(velocity, velocity, acceleration, acceleration),
            True,
            True,
        )
        await task_1
        await task_2

    async def wait(self, ms: int) -> None:
        await self._loop.run_in_executor(
            None, self.dobotApiInterface.set_wait_cmd, tagWAITCmd(ms)
        )

    async def pose(
        self,
    ) -> Tuple[float, float, float, float, float, float, float, float]:
        pos: tagPose = await self._loop.run_in_executor(
            None, self.dobotApiInterface.get_pose
        )
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
