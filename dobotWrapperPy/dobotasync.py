from .dobotapi import DobotApi
from .dobotConnection import DobotConnection 
import warnings
import struct
from .enums.alarm import Alarm
from .enums.ptpMode import PTPMode
from .enums.tagVersionColorSensorAndIR import TagVersionColorSensorAndIR
from .enums.tagVersionRail import tagVersionRail
from .enums.realTimeTrack import RealTimeTrack
from .enums.CPMode import CPMode
from .enums.level import Level
from .enums.IOFunction import IOFunction
from .enums.EMotorIndex import EMotorIndex
from .enums.jogCmd import JogCmd
from .enums.jogMode import JogMode
from .message import Message
from .paramsStructures import (
    tagIOMultiplexing,
    tagWithL,
    tagDevice,
    tagPTPCommonParams,
    tagPTPCoordinateParams,
    tagWAITCmd,
    tagPose,
    tagPTPCmd,
    tagPTPWithLCmd,
    tagHomeCmd,
    tagCPParams,
    tagCPCmd,
    tagIODO,
    tagIODI,
    tagIOPWM,
    tagEMOTOR,
    tagJOGCmd,
    tagARCCmd,
)
import asyncio
from typing import Tuple, Optional, Set, Any
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
        signal.signal(signal.SIGINT, self._on_sigint)
        self._endEffectorType = None

    async def _on_sigint(self, signum: int, frame: Optional[Any]) -> None:
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

    async def move_xyz_linear(self, x: float, y: float, z: float, r: float) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_ptp_cmd,
            tagPTPCmd(PTPMode.MOVL_XYZ, x, y, z, r),
            True,
            True,
        )

    async def move_xyz_joint(self, x: float, y: float, z: float, r: float) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_ptp_cmd,
            tagPTPCmd(PTPMode.MOVJ_XYZ, x, y, z, r),
            True,
            True,
        )

    async def move_relative_xyz_linear(
        self, delta_x: float, delta_y: float, delta_z: float, delta_r: float
    ) -> None:
        (x, y, z, r, _, _, _, _) = await self.pose()
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_ptp_cmd,
            tagPTPCmd(
                PTPMode.MOVL_XYZ, x + delta_x, y + delta_y, z + delta_z, r + delta_r
            ),
            True,
            True,
        )

    async def move_relative_xyz_joint(
        self, delta_x: float, delta_y: float, delta_z: float, delta_r: float
    ) -> None:
        (x, y, z, r, _, _, _, _) = await self.pose()
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_ptp_cmd,
            tagPTPCmd(
                PTPMode.MOVJ_XYZ, x + delta_x, y + delta_y, z + delta_z, r + delta_r
            ),
            True,
            True,
        )

    async def jump_relative_xyz(
        self, delta_x: float, delta_y: float, delta_z: float, delta_r: float
    ) -> None:
        (x, y, z, r, _, _, _, _) = await self.pose()
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_ptp_cmd,
            tagPTPCmd(
                PTPMode.JUMP_XYZ, x + delta_x, y + delta_y, z + delta_z, r + delta_r
            ),
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

    # Rail Movement

    async def move_xyz_rail_linear(
        self, x: float, y: float, z: float, r: float, rail: float
    ) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_ptp_with_rail_cmd,
            tagPTPWithLCmd(PTPMode.MOVL_XYZ, x, y, z, r, rail),
            True,
            True,
        )

    async def move_xyz_rail_joint(
        self, x: float, y: float, z: float, r: float, rail: float
    ) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_ptp_with_rail_cmd,
            tagPTPWithLCmd(PTPMode.MOVJ_XYZ, x, y, z, r, rail),
            True,
            True,
        )

    async def move_relative_xyz_rail_linear(
        self,
        delta_x: float,
        delta_y: float,
        delta_z: float,
        delta_r: float,
        delta_rail: float,
    ) -> None:
        (x, y, z, r, _, _, _, _) = await self.pose()
        rail = await self.pose_rail()
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_ptp_with_rail_cmd,
            tagPTPWithLCmd(
                PTPMode.MOVL_XYZ,
                x + delta_x,
                y + delta_y,
                z + delta_z,
                r + delta_r,
                rail + delta_rail,
            ),
            True,
            True,
        )

    async def move_relative_xyz_rail_joint(
        self,
        delta_x: float,
        delta_y: float,
        delta_z: float,
        delta_r: float,
        delta_rail: float,
    ) -> None:
        (x, y, z, r, _, _, _, _) = await self.pose()
        rail = await self.pose_rail()
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_ptp_with_rail_cmd,
            tagPTPWithLCmd(
                PTPMode.MOVJ_XYZ,
                x + delta_x,
                y + delta_y,
                z + delta_z,
                r + delta_r,
                rail + delta_rail,
            ),
            True,
            True,
        )

    async def jump_relative_xyz_rail(
        self,
        delta_x: float,
        delta_y: float,
        delta_z: float,
        delta_r: float,
        delta_rail: float,
    ) -> None:
        (x, y, z, r, _, _, _, _) = await self.pose()
        rail = await self.pose_rail()
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_ptp_with_rail_cmd,
            tagPTPWithLCmd(
                PTPMode.JUMP_XYZ,
                x + delta_x,
                y + delta_y,
                z + delta_z,
                r + delta_r,
                rail + delta_rail,
            ),
            True,
            True,
        )

    async def jump_rail(
        self, x: float, y: float, z: float, r: float, rail: float
    ) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_ptp_with_rail_cmd,
            tagPTPWithLCmd(PTPMode.JUMP_MOVL_XYZ, x, y, z, r, rail),
            True,
            True,
        )

    async def clear_alarms(self) -> None:
        await self._loop.run_in_executor(
            None, self.dobotApiInterface.clear_all_alarms_state
        )

    async def get_alarms(self) -> Set[Alarm]:
        return await self._loop.run_in_executor(
            None, self.dobotApiInterface.get_active_alarms
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
            None, self.dobotApiInterface.set_wait_cmd, tagWAITCmd(ms), True, True
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

    async def pose_rail(self) -> float:
        pos = await self._loop.run_in_executor(
            None, self.dobotApiInterface.get_pose_rail
        )
        return pos

    async def home(self) -> None:
        await self._loop.run_in_executor(
            None, self.dobotApiInterface.set_home_cmd, tagHomeCmd(0), True, True
        )

    async def get_ir_value(self, port: int) -> bool:
        return await self._loop.run_in_executor(
            None, self.dobotApiInterface.get_ir_switch, port
        )

    async def set_ir_params(
        self, port: int, version: TagVersionColorSensorAndIR
    ) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_ir_switch,
            tagDevice(True, port, version),
            True,
            True,
        )

    async def get_device_serial_number(self) -> str:
        return await self._loop.run_in_executor(
            None, self.dobotApiInterface.get_device_sn
        )

    async def get_device_id(self) -> Tuple[int, int, int]:

        return await self._loop.run_in_executor(
            None, self.dobotApiInterface.get_device_id
        )

    async def get_device_name(self) -> str:
        return await self._loop.run_in_executor(
            None, self.dobotApiInterface.get_device_name
        )

    async def get_device_rail_capability(self) -> bool:
        return await self._loop.run_in_executor(
            None, self.dobotApiInterface.get_device_rail_capability
        )

    async def get_device_time(self) -> int:
        return await self._loop.run_in_executor(
            None, self.dobotApiInterface.get_device_time
        )

    async def get_device_version(self) -> Tuple[int, int, int]:
        return await self._loop.run_in_executor(
            None, self.dobotApiInterface.get_device_version
        )

    async def set_device_serial_number(self, serial_number: str) -> None:
        await self._loop.run_in_executor(
            None, self.dobotApiInterface.set_device_sn, serial_number
        )

    async def set_device_name(self, name: str) -> None:
        await self._loop.run_in_executor(
            None, self.dobotApiInterface.set_device_name, name
        )

    async def set_device_rail_capability(
        self, name: str, enable: bool, version: tagVersionRail
    ) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_device_rail_capability,
            tagWithL(enable, version),
        )

    async def set_lost_step_error(self, error: float) -> None:
        await self._loop.run_in_executor(
            None, self.dobotApiInterface.set_lost_step_cmd, True, True
        )

    async def set_lost_step_command(self, threshold: float) -> None:
        await self._loop.run_in_executor(
            None, self.dobotApiInterface.set_lost_step_cmd, True, True
        )

    async def set_continous_trajectory_parameters(
        self, acceleration: float, realTime: RealTimeTrack
    ) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_cp_params,
            tagCPParams(acceleration, acceleration, acceleration, realTime),
            True,
            True,
        )

    async def move_with_continous_trajectory_relative(
        self, delta_x: float, delta_y: float, delta_z: float
    ) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_cp_cmd,
            tagCPCmd(CPMode.RELATIVE, delta_x, delta_y, delta_z, 10),
            True,
            True,
        )

    async def move_with_continous_trajectory_absolute(
        self, x: float, y: float, z: float
    ) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_cp_cmd,
            tagCPCmd(CPMode.ABSOLUTE, x, y, z, 10),
            True,
            True,
        )

    async def move_with_continous_trajectory_laser_relative(
        self, delta_x: float, delta_y: float, delta_z: float, power: float
    ) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_cp_le_cmd,
            tagCPCmd(CPMode.RELATIVE, delta_x, delta_y, delta_z, power),
            True,
            True,
        )

    async def set_angle_static_error(
        self, rear_arm_angle: float, front_arm_angle: float
    ) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_angle_sensor_static_error,
            rear_arm_angle,
            front_arm_angle,
        )

    async def get_angle_static_error(self) -> Tuple[float, float]:
        return await self._loop.run_in_executor(
            None, self.dobotApiInterface.get_angle_sensor_static_error
        )

    async def set_pin_purpose(self, address: int, purpose: IOFunction) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_io_multiplexing,
            tagIOMultiplexing(address, purpose),
            True,
            True,
        )

    async def get_pin_purpose(self, address: int) -> IOFunction:
        return (
            await self._loop.run_in_executor(
                None, self.dobotApiInterface.get_io_multiplexing, address
            )
        ).multiplex

    async def set_pin_output(self, address: int, level: Level) -> None:
        await self._loop.run_in_executor(
            None, self.dobotApiInterface.set_io_do, tagIODO(address, level), True, True
        )

    async def get_pin_output(self, address: int) -> Level:
        return await self._loop.run_in_executor(
            None, self.dobotApiInterface.get_io_do, address
        )

    async def get_pin_input(self, address: int) -> Level:
        return await self._loop.run_in_executor(
            None, self.dobotApiInterface.get_io_di, address
        )

    async def get_adc(self, address: int) -> int:
        return await self._loop.run_in_executor(
            None, self.dobotApiInterface.get_io_adc, address
        )

    async def set_pwm(self, address: int, frequency: float, cycle: float) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_io_pwm,
            tagIOPWM(address, frequency, cycle),
            True,
            True,
        )

    async def get_pwm(self, address: int) -> Tuple[float, float]:
        result = await self._loop.run_in_executor(
            None, self.dobotApiInterface.get_io_pwm, address
        )
        return (result.frequency, result.dutyCycle)

    async def set_motor(self, address: EMotorIndex, enable: bool, speed: float) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_e_motor,
            tagEMOTOR(address, enable, speed),
            True,
            True,
        )

    async def set_color_sensor(
        self, enable: bool, port: int, version: TagVersionColorSensorAndIR
    ) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_color_sensor,
            tagDevice(enable, port, version),
            True,
            True,
        )

    async def get_color_sensor(self, port: int) -> Tuple[int, int, int]:
        return await self._loop.run_in_executor(
            None, self.dobotApiInterface.get_color_sensor, port
        )

    async def move_joystick_positive_x(self) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_jog_cmd,
            tagJOGCmd(JogMode.COORDINATE, JogCmd.AP_DOWN),
            True,
            True,
        )

    async def move_joystick_negative_x(self) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_jog_cmd,
            tagJOGCmd(JogMode.COORDINATE, JogCmd.AN_DOWN),
            True,
            True,
        )

    async def move_joystick_positive_y(self) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_jog_cmd,
            tagJOGCmd(JogMode.COORDINATE, JogCmd.BP_DOWN),
            True,
            True,
        )

    async def move_joystick_negative_y(self) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_jog_cmd,
            tagJOGCmd(JogMode.COORDINATE, JogCmd.BN_DOWN),
            True,
            True,
        )

    async def move_joystick_positive_z(self) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_jog_cmd,
            tagJOGCmd(JogMode.COORDINATE, JogCmd.CP_DOWN),
            True,
            True,
        )

    async def move_joystick_negative_z(self) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_jog_cmd,
            tagJOGCmd(JogMode.COORDINATE, JogCmd.CN_DOWN),
            True,
            True,
        )

    async def move_joystick_positive_r(self) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_jog_cmd,
            tagJOGCmd(JogMode.COORDINATE, JogCmd.DP_DOWN),
            True,
            True,
        )

    async def move_joystick_negative_r(self) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_jog_cmd,
            tagJOGCmd(JogMode.COORDINATE, JogCmd.DN_DOWN),
            True,
            True,
        )

    async def move_in_circle(
        self,
        relative_x: float,
        relative_y: float,
        relative_z: float,
        relative_r: float,
        end_x: float,
        end_y: float,
        end_z: float,
        end_r: float,
    ) -> None:
        await self._loop.run_in_executor(
            None,
            self.dobotApiInterface.set_arc_cmd,
            tagARCCmd(
                tagARCCmd.Point(relative_x, relative_y, relative_z, relative_r),
                tagARCCmd.Point(end_x, end_y, end_z, end_r),
            ),
            True,
            True,
        )
