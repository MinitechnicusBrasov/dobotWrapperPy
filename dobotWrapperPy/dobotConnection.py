# mypy: disable-error-code="import"
import serial
from typing import Optional


class DobotConnection:

    serial_conn: serial.Serial

    def __init__(
        self, port: Optional[str] = None, serial_conn: Optional[serial.Serial] = None
    ) -> None:
        if port is None and serial_conn is None:
            raise TypeError(
                "Please provide the constructor with port or with the serial connection"
            )

        if serial_conn is None:
            serial_conn = serial.Serial(
                port,
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
            )

        self.serial_conn = serial_conn

    def __del(self) -> None:
        self.serial_conn.close()
