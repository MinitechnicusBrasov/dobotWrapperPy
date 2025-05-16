import enum


class ControlValues(enum.Enum):
    Zero = 0x00
    ReadWrite = 0x01
    isQueued = 0x02
    Both = 0x03
