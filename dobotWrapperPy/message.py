from typing import Optional
from .enums.ControlValues import ControlValues
from .enums.CommunicationProtocolIDs import CommunicationProtocolIDs
import warnings


class Message:
    header: bytes
    len: int
    ctrl: ControlValues
    params: bytes
    checksum: Optional[int]
    id: CommunicationProtocolIDs

    def __init__(self, b: Optional[bytes] = None) -> None:
        if b is None:
            self.header = bytes([0xAA, 0xAA])
            self.len = 0x00
            self.ctrl = ControlValues.Zero
            self.params = bytearray([])
            self.checksum = None
        else:
            self.header = b[0:2]
            self.len = b[2]
            self.id = CommunicationProtocolIDs(b[3])
            self.ctrl = ControlValues(b[4])
            self.params = b[5:-1]
            self.checksum = b[-1:][0]

    def __repr__(self) -> str:
        return "Message()"

    def __str__(self) -> str:
        self.refresh()
        hexHeader = " ".join("%02x" % b for b in self.header)
        hexParams = " ".join("%02x" % b for b in self.params)
        ret = "%s:%d:%d:%d:%s:%s" % (
            hexHeader,
            self.len,
            self.id.value,
            self.ctrl.value,
            hexParams,
            self.checksum,
        )
        return ret.upper()

    def refresh(self) -> None:

        # if self.checksum is not None:
        #     warnings.warn("Checksum not recalculated")
        #     return

        if not all(isinstance(p, int) for p in self.params):
            raise TypeError("All params must be integers")

        total = self.id.value + self.ctrl.value + sum(self.params)
        total %= 256

        self.checksum = (256 - total) % 255

        self.len = 0x02 + len(self.params)

        # self.ctrl = self.ctrl
        # self.id = self.id
        # if self.checksum is None:
        #     self.checksum = self.id.value + self.ctrl.value
        #     for i in range(len(self.params)):
        #         if isinstance(self.params[i], int):
        #             self.checksum += self.params[i]
        #         else:
        #             raise TypeError(f"Param: {self.params[i]} is not an int")
        #     self.checksum = self.checksum % 256
        #     self.checksum = 2**8 - self.checksum
        #     self.checksum = self.checksum % 255  #!TODO verify this
        #     self.len = 0x02 + len(self.params)

    """
    Verifies whether the stored checksum is valid for the current ID, CTRL, and parameters.
    Returns:
        bool: True if checksum is valid, False otherwise.
    """

    def verify_checksum(self) -> bool:

        if not all(isinstance(p, int) for p in self.params):
            raise TypeError("All params must be integers")

        total = self.id.value + self.ctrl.value + sum(self.params)
        total %= 256
        expected_checksum = (256 - total) % 255

        return self.checksum == expected_checksum

    def bytes(self) -> bytes:
        self.refresh()
        if self.checksum is None:
            raise TypeError("Checksum is None")

        if len(self.params) > 0:
            command = bytearray([0xAA, 0xAA, self.len, self.id.value, self.ctrl.value])
            command.extend(self.params)

            command.append(self.checksum)
        else:
            command = bytearray(
                [0xAA, 0xAA, self.len, self.id.value, self.ctrl.value, self.checksum]
            )
        return command
