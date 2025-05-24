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
from typing import Tuple, Optional
from enum import Enum
import signal
import sys


class EndEffectorType(Enum):
    CUP = 0
    GRIPPER = 1
    LASER = 2


class DobotAsync:
    dobotApiInterface: DobotApi
    _loop: asyncio.AbstractEventLoop
    _endEffectorType: Optional[EndEffectorType]

    def __init__(self, port: str, verbose: bool = False) -> None:
        conn = DobotConnection(port=port)
        self.dobotApiInterface = DobotApi(conn, verbose)
        self._loop = asyncio.get_running_loop()
        self._loop.add_signal_handler(signal.SIGINT, self._on_sigint)
        self._endEffectorType = None

    async def _on_sigint(self) -> None:
        print("SIGINT received. Force Stopping robot...")
        self.force_stop()
        match self._endEffectorType:
            case EndEffectorType.CUP:
                await self.suck(False)
            case EndEffectorType.GRIPPER:
                await self.grip(False)
            case EndEffectorType.LASER:
                await self.laser(False)
        # Exit the program immediately — no further code runs
        sys.exit(130)  # Standard exit code for Ctrl+C

    def force_stop(self) -> None:
        self.dobotApiInterface.set_queued_cmd_force_stop_exec()

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
        self._endEffectorType = EndEffectorType.CUP

    async def grip(self, enable: bool) -> None:
        await self._loop.run_in_executor(
            None, self.dobotApiInterface.set_end_effector_gripper, enable, True, True
        )
        self._endEffectorType = EndEffectorType.GRIPPER

    async def laser(self, enable: bool) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_end_effector_laser,
            True,
            enable,
            True,
            True,
        )
        self._endEffectorType = EndEffectorType.LASER

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
