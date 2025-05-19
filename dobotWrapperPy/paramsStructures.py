from dataclasses import dataclass
from .enums.tagVersionRail import tagVersionRail
from .enums.jogMode import JogMode
from .enums.jogCmd import JogCmd
from .enums.ptpMode import PTPMode
from .enums.realTimeTrack import RealTimeTrack
from .enums.CPMode import CPMode
from .enums.triggerMode import TriggerMode
from .enums.triggerCondition import TriggerCondition
from .enums.IOFunction import IOFunction
from .enums.level import Level
from .enums.EMotorIndex import EMotorIndex
from .enums.tagVersionColorSensorAndIR import TagVersionColorSensorAndIR
from typing import List
import struct


@dataclass
class tagWithL:
    """Represents a tag with rail information and version."""

    is_with_rail: bool
    version: tagVersionRail

    def pack(self) -> bytes:
        """
        Packs the tagWithL object into a bytes sequence.

        Packs a boolean (as a byte) and the version value (as a byte)
        into a byte string. Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (2 bytes).
        """
        # Format: < (little-endian), B (unsigned char - 1 byte), B (unsigned char - 1 byte)
        # Total size = 1 + 1 = 2 bytes
        return struct.pack("<BB", 1 if self.is_with_rail else 0, self.version.value)

    @classmethod
    def unpack(cls, data: bytes) -> "tagWithL":
        """
        Unpacks a bytes sequence into a tagWithL object.

        Args:
            data: The bytes to unpack, expected to be 2 bytes.

        Returns:
            A tagWithL object.

        Raises:
            struct.error: If the input bytes are not the expected size (2 bytes).
            ValueError: If the unpacked version value does not correspond to a valid tagVersionRail enum member.
        """
        format_string = "<BB"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_byte_rail, unpacked_byte_version = struct.unpack(format_string, data)

        is_with_rail = unpacked_byte_rail == 1

        try:
            version = tagVersionRail(unpacked_byte_version)
        except ValueError:
            raise ValueError(
                f"Invalid version value encountered: {unpacked_byte_version}"
            )

        return cls(is_with_rail=is_with_rail, version=version)


@dataclass
class tagWithLReturn:
    """Represents a return tag with rail information."""

    is_with_rail: bool

    def pack(self) -> bytes:
        """
        Packs the tagWithLReturn object into a bytes sequence.

        Packs a boolean (as a byte) into a byte string.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (1 byte).
        """
        # Format: < (little-endian), B (unsigned char - 1 byte)
        # Total size = 1 byte
        return struct.pack("<B", 1 if self.is_with_rail else 0)

    @classmethod
    def unpack(cls, data: bytes) -> "tagWithLReturn":
        """
        Unpacks bytes into a tagWithLReturn object.

        Args:
            data: The bytes to unpack, expected to be a single byte.
            cls: The class itself (used for creating an instance).

        Returns:
            A tagWithLReturn object.

        Raises:
            struct.error: If the input bytes are not the expected size (1 byte).
        """
        format_string = "<B"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_byte = struct.unpack(format_string, data)[0]
        is_with_rail = unpacked_byte == 1
        return cls(is_with_rail=is_with_rail)


@dataclass
class tagPose:
    x: float
    y: float
    z: float
    r: float
    # The basement, rear arm, forearm,EndEffector - Expecting 4 joint angles
    jointAngle: List[float]  # Expecting a list of 4 floats

    def pack(self) -> bytes:
        """
        Packs the tagPose object into a bytes sequence.

        Packs 4 floats (x, y, z, r) and 4 joint angles (floats) into a byte string.
        Assumes jointAngle contains exactly 4 float values.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data.

        Raises:
            ValueError: If jointAngle does not contain exactly 4 elements.
        """
        if len(self.jointAngle) != 4:
            raise ValueError(
                f"jointAngle must contain exactly 4 elements, but got {len(self.jointAngle)}"
            )

        # Format: < (little-endian), 8 * d (double, 8 bytes each)
        # Total size = 8 * 8 = 64 bytes
        format_string = "<ffffffff"
        return struct.pack(
            format_string,
            self.x,
            self.y,
            self.z,
            self.r,
            *self.jointAngle,  # Unpack the list elements
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagPose":
        """
        Unpacks a bytes sequence into a tagPose object.

        Args:
            data: The bytes to unpack, expected to be 64 bytes (8 doubles).

        Returns:
            A tagPose object.

        Raises:
            struct.error: If the input bytes are not the expected size (64 bytes).
        """
        # Format: < (little-endian), 8 * d (double, 8 bytes each)
        # Expected size = 8 * 8 = 64 bytes
        format_string = "<ffffffff"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        x, y, z, r = unpacked_data[:4]
        jointAngle = list(unpacked_data[4:])  # Get the last 4 elements as a list

        return cls(x=x, y=y, z=z, r=r, jointAngle=jointAngle)


@dataclass
class tagHomeParams:
    """Represents home parameters with x, y, z, and r coordinates."""

    x: float
    y: float
    z: float
    r: float

    def pack(self) -> bytes:
        """
        Packs the tagHomeParams object into a bytes sequence.

        Packs 4 floats (x, y, z, r) into a byte string.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (32 bytes).
        """
        # Format: < (little-endian), 4 * d (double, 8 bytes each)
        # Total size = 4 * 8 = 32 bytes
        format_string = "<ffff"
        return struct.pack(
            format_string,
            self.x,
            self.y,
            self.z,
            self.r,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagHomeParams":
        """
        Unpacks a bytes sequence into a tagHomeParams object.

        Args:
            data: The bytes to unpack, expected to be 32 bytes (4 doubles).

        Returns:
            A tagHomeParams object.

        Raises:
            struct.error: If the input bytes are not the expected size (32 bytes).
        """
        # Format: < (little-endian), 4 * d (double, 8 bytes each)
        # Expected size = 4 * 8 = 32 bytes
        format_string = "<ffff"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        x, y, z, r = unpacked_data

        return cls(
            x=x,
            y=y,
            z=z,
            r=r,
        )


@dataclass
class tagHomeCmd:
    """Represents a home command with a reserved integer field."""

    # uint32
    reserved: int

    def pack(self) -> bytes:
        """
        Packs the tagHomeCmd object into a bytes sequence.

        Packs the reserved integer (uint32) into a byte string.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (4 bytes).
        """
        # Format: < (little-endian), I (unsigned int - 4 bytes)
        # Total size = 4 bytes
        format_string = "<I"
        return struct.pack(
            format_string,
            self.reserved,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagHomeCmd":
        """
        Unpacks a bytes sequence into a tagHomeCmd object.

        Args:
            data: The bytes to unpack, expected to be 4 bytes (1 unsigned int).

        Returns:
            A tagHomeCmd object.

        Raises:
            struct.error: If the input bytes are not the expected size (4 bytes).
        """
        # Format: < (little-endian), I (unsigned int - 4 bytes)
        # Expected size = 4 bytes
        format_string = "<I"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)[0]

        # Unpack the tuple into the respective fields
        reserved = unpacked_data

        return cls(
            reserved=reserved,
        )


@dataclass
class tagAutoLevelingParams:
    """Represents auto-leveling parameters."""

    # uint8
    isAutoLeveling: bool
    # double
    accuracy: float

    def pack(self) -> bytes:
        """
        Packs the tagAutoLevelingParams object into a bytes sequence.

        Packs the boolean (as a uint8) and the accuracy (double) into a byte string.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (1 byte + 8 bytes = 9 bytes).
        """
        # Format: < (little-endian), B (unsigned char - 1 byte), d (double - 8 bytes)
        # Total size = 1 + 8 = 9 bytes
        format_string = "<Bf"
        return struct.pack(
            format_string,
            1 if self.isAutoLeveling else 0,  # Pack boolean as 1 or 0
            self.accuracy,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagAutoLevelingParams":
        """
        Unpacks a bytes sequence into a tagAutoLevelingParams object.

        Args:
            data: The bytes to unpack, expected to be 9 bytes (1 uint8 + 1 double).

        Returns:
            A tagAutoLevelingParams object.

        Raises:
            struct.error: If the input bytes are not the expected size (9 bytes).
        """
        # Format: < (little-endian), B (unsigned char - 1 byte), d (double - 8 bytes)
        # Expected size = 1 + 8 = 9 bytes
        format_string = "<Bf"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_byte_is_autoleveling, unpacked_double_accuracy = struct.unpack(
            format_string, data
        )

        is_autoleveling = unpacked_byte_is_autoleveling == 1
        accuracy = unpacked_double_accuracy

        return cls(
            isAutoLeveling=is_autoleveling,
            accuracy=accuracy,
        )


@dataclass
class tagEndEffectorParams:
    """Represents end effector parameters with bias coordinates."""

    xBias: float
    yBias: float
    zBias: float

    def pack(self) -> bytes:
        """
        Packs the tagEndEffectorParams object into a bytes sequence.

        Packs 3 floats (xBias, yBias, zBias) into a byte string.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (24 bytes).
        """
        # Format: < (little-endian), 3 * d (double, 8 bytes each)
        # Total size = 3 * 8 = 24 bytes
        format_string = "<ddd"
        return struct.pack(
            format_string,
            self.xBias,
            self.yBias,
            self.zBias,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagEndEffectorParams":
        """
        Unpacks a bytes sequence into a tagEndEffectorParams object.

        Args:
            data: The bytes to unpack, expected to be 24 bytes (3 doubles).

        Returns:
            A tagEndEffectorParams object.

        Raises:
            struct.error: If the input bytes are not the expected size (24 bytes).
        """
        # Format: < (little-endian), 3 * d (double, 8 bytes each)
        # Expected size = 3 * 8 = 24 bytes
        format_string = "<ddd"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        xBias, yBias, zBias = unpacked_data

        return cls(
            xBias=xBias,
            yBias=yBias,
            zBias=zBias,
        )


@dataclass
class tagJOGJointParams:
    """Represents JOG joint parameters with velocity and acceleration."""

    # Joint velocity of 4 axis
    velocity: List[float]
    # Joint acceleration of 4 axis
    acceleration: List[float]

    def pack(self) -> bytes:
        """
        Packs the tagJOGJointParams object into a bytes sequence.

        Packs 4 velocity floats and 4 acceleration floats into a byte string.
        Assumes both velocity and acceleration lists contain exactly 4 float values.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (64 bytes).

        Raises:
            ValueError: If velocity or acceleration lists do not contain exactly 4 elements.
        """
        if len(self.velocity) != 4 or len(self.acceleration) != 4:
            raise ValueError(
                f"velocity and acceleration lists must contain exactly 4 elements each."
            )

        # Format: < (little-endian), 8 * d (double, 8 bytes each)
        # Total size = 8 * 8 = 64 bytes
        format_string = "<ffffffff"
        return struct.pack(
            format_string,
            *self.velocity,  # Unpack the velocity list elements
            *self.acceleration,  # Unpack the acceleration list elements
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagJOGJointParams":
        """
        Unpacks a bytes sequence into a tagJOGJointParams object.

        Args:
            data: The bytes to unpack, expected to be 64 bytes (8 doubles).

        Returns:
            A tagJOGJointParams object.

        Raises:
            struct.error: If the input bytes are not the expected size (64 bytes).
        """
        # Format: < (little-endian), 8 * d (double, 8 bytes each)
        # Expected size = 8 * 8 = 64 bytes
        format_string = "<ffffffff"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        velocity = list(unpacked_data[:4])  # Get the first 4 elements for velocity
        acceleration = list(
            unpacked_data[4:]
        )  # Get the last 4 elements for acceleration

        return cls(
            velocity=velocity,
            acceleration=acceleration,
        )


@dataclass
class tagJOGCoordinateParams:
    """Represents JOG coordinate parameters with velocity and acceleration."""

    # Coordinate velocity of 4 axis (x,y,z,r)
    velocity: List[float]
    # Coordinate acceleration of 4 axis (x,y,z,r)
    acceleration: List[float]

    def pack(self) -> bytes:
        """
        Packs the tagJOGCoordinateParams object into a bytes sequence.

        Packs 4 velocity floats and 4 acceleration floats into a byte string.
        Assumes both velocity and acceleration lists contain exactly 4 float values.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (64 bytes).

        Raises:
            ValueError: If velocity or acceleration lists do not contain exactly 4 elements.
        """
        if len(self.velocity) != 4 or len(self.acceleration) != 4:
            raise ValueError(
                f"velocity and acceleration lists must contain exactly 4 elements each."
            )

        # Format: < (little-endian), 8 * d (double, 8 bytes each)
        # Total size = 8 * 8 = 64 bytes
        format_string = "<ffffffff"
        return struct.pack(
            format_string,
            *self.velocity,  # Unpack the velocity list elements
            *self.acceleration,  # Unpack the acceleration list elements
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagJOGCoordinateParams":
        """
        Unpacks a bytes sequence into a tagJOGCoordinateParams object.

        Args:
            data: The bytes to unpack, expected to be 64 bytes (8 doubles).

        Returns:
            A tagJOGCoordinateParams object.

        Raises:
            struct.error: If the input bytes are not the expected size (64 bytes).
        """
        # Format: < (little-endian), 8 * d (double, 8 bytes each)
        # Expected size = 8 * 8 = 64 bytes
        format_string = "<ffffffff"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        velocity = list(unpacked_data[:4])  # Get the first 4 elements for velocity
        acceleration = list(
            unpacked_data[4:]
        )  # Get the last 4 elements for acceleration

        return cls(
            velocity=velocity,
            acceleration=acceleration,
        )


@dataclass
class tagJOGCommonParams:
    """Represents common JOG parameters with velocity and acceleration ratios."""

    velocityRatio: float
    accelerationRatio: float

    def pack(self) -> bytes:
        """
        Packs the tagJOGCommonParams object into a bytes sequence.

        Packs the velocityRatio and accelerationRatio (floats) into a byte string.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (16 bytes).
        """
        # Format: < (little-endian), 2 * d (double, 8 bytes each)
        # Total size = 2 * 8 = 16 bytes
        format_string = "<ff"
        return struct.pack(
            format_string,
            self.velocityRatio,
            self.accelerationRatio,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagJOGCommonParams":
        """
        Unpacks a bytes sequence into a tagJOGCommonParams object.

        Args:
            data: The bytes to unpack, expected to be 16 bytes (2 doubles).

        Returns:
            A tagJOGCommonParams object.

        Raises:
            struct.error: If the input bytes are not the expected size (16 bytes).
        """
        # Format: < (little-endian), 2 * d (double, 8 bytes each)
        # Expected size = 2 * 8 = 16 bytes
        format_string = "<ff"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        velocityRatio, accelerationRatio = unpacked_data

        return cls(
            velocityRatio=velocityRatio,
            accelerationRatio=accelerationRatio,
        )


@dataclass
class tagJOGCmd:
    """Represents a JOG command."""

    # uint8 - Represents JogMode
    isJoint: JogMode
    # uint8 - Represents JogCmd
    cmd: JogCmd

    def pack(self) -> bytes:
        """
        Packs the tagJOGCmd object into a bytes sequence.

        Packs the isJoint (JogMode value) and cmd (JogCmd value) into a byte string.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (2 bytes).
        """
        # Format: < (little-endian), B (unsigned char - 1 byte), B (unsigned char - 1 byte)
        # Total size = 1 + 1 = 2 bytes
        format_string = "<BB"
        return struct.pack(
            format_string,
            self.isJoint.value,  # Pack the enum value
            self.cmd.value,  # Pack the enum value
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagJOGCmd":
        """
        Unpacks a bytes sequence into a tagJOGCmd object.

        Args:
            data: The bytes to unpack, expected to be 2 bytes (2 uint8s).

        Returns:
            A tagJOGCmd object.

        Raises:
            struct.error: If the input bytes are not the expected size (2 bytes).
            ValueError: If the unpacked byte values do not correspond to valid enum members.
        """
        format_string = "<BB"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_byte_is_joint, unpacked_byte_cmd = struct.unpack(format_string, data)

        try:
            is_joint = JogMode(unpacked_byte_is_joint)
        except ValueError:
            raise ValueError(
                f"Invalid JogMode value encountered: {unpacked_byte_is_joint}"
            )

        try:
            cmd = JogCmd(unpacked_byte_cmd)
        except ValueError:
            raise ValueError(f"Invalid JogCmd value encountered: {unpacked_byte_cmd}")

        return cls(
            isJoint=is_joint,
            cmd=cmd,
        )


@dataclass
class tagJOGLParams:
    """Represents JOG linear parameters with velocity and acceleration."""

    velocity: float
    acceleration: float

    def pack(self) -> bytes:
        """
        Packs the tagJOGLParams object into a bytes sequence.

        Packs the velocity and acceleration (floats) into a byte string.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (16 bytes).
        """
        # Format: < (little-endian), 2 * d (double, 8 bytes each)
        # Total size = 2 * 8 = 16 bytes
        format_string = "<ff"
        return struct.pack(
            format_string,
            self.velocity,
            self.acceleration,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagJOGLParams":
        """
        Unpacks a bytes sequence into a tagJOGLParams object.

        Args:
            data: The bytes to unpack, expected to be 16 bytes (2 doubles).

        Returns:
            A tagJOGLParams object.

        Raises:
            struct.error: If the input bytes are not the expected size (16 bytes).
        """
        # Format: < (little-endian), 2 * d (double, 8 bytes each)
        # Expected size = 2 * 8 = 16 bytes
        format_string = "<ff"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        velocity, acceleration = unpacked_data

        return cls(
            velocity=velocity,
            acceleration=acceleration,
        )


@dataclass
class tagPTPJointParams:
    """Represents PTP joint parameters with velocity and acceleration."""

    # Joint velocity of 4 axis
    velocity: List[float]
    # Joint acceleration of 4 axis
    acceleration: List[float]

    def pack(self) -> bytes:
        """
        Packs the tagPTPJointParams object into a bytes sequence.

        Packs 4 velocity floats and 4 acceleration floats into a byte string.
        Assumes both velocity and acceleration lists contain exactly 4 float values.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (64 bytes).

        Raises:
            ValueError: If velocity or acceleration lists do not contain exactly 4 elements.
        """
        if len(self.velocity) != 4 or len(self.acceleration) != 4:
            raise ValueError(
                f"velocity and acceleration lists must contain exactly 4 elements each."
            )

        # Format: < (little-endian), 8 * d (double, 8 bytes each)
        # Total size = 8 * 8 = 64 bytes
        format_string = "<ffffffff"
        return struct.pack(
            format_string,
            *self.velocity,  # Unpack the velocity list elements
            *self.acceleration,  # Unpack the acceleration list elements
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagPTPJointParams":
        """
        Unpacks a bytes sequence into a tagPTPJointParams object.

        Args:
            data: The bytes to unpack, expected to be 64 bytes (8 doubles).

        Returns:
            A tagPTPJointParams object.

        Raises:
            struct.error: If the input bytes are not the expected size (64 bytes).
        """
        # Format: < (little-endian), 8 * d (double, 8 bytes each)
        # Expected size = 8 * 8 = 64 bytes
        format_string = "<ffffffff"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        velocity = list(unpacked_data[:4])  # Get the first 4 elements for velocity
        acceleration = list(
            unpacked_data[4:]
        )  # Get the last 4 elements for acceleration

        return cls(
            velocity=velocity,
            acceleration=acceleration,
        )


@dataclass
class tagPTPCoordinateParams:
    """Represents PTP coordinate parameters with velocity and acceleration."""

    # Coordinate velocity (xyz)
    xyzVelocity: float
    # Coordinate velocity (r)
    rVelocity: float
    # Coordinate acceleration (xyz)
    xyzAcceleration: float
    # Coordinate acceleration (r)
    rAcceleration: float

    def pack(self) -> bytes:
        """
        Packs the tagPTPCoordinateParams object into a bytes sequence.

        Packs the xyzVelocity, rVelocity, xyzAcceleration, and rAcceleration
        (floats) into a byte string. Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (32 bytes).
        """
        # Format: < (little-endian), 4 * d (double, 8 bytes each)
        # Total size = 4 * 8 = 32 bytes
        format_string = "<ffff"
        return struct.pack(
            format_string,
            self.xyzVelocity,
            self.rVelocity,
            self.xyzAcceleration,
            self.rAcceleration,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagPTPCoordinateParams":
        """
        Unpacks a bytes sequence into a tagPTPCoordinateParams object.

        Args:
            data: The bytes to unpack, expected to be 32 bytes (4 doubles).

        Returns:
            A tagPTPCoordinateParams object.

        Raises:
            struct.error: If the input bytes are not the expected size (32 bytes).
        """
        # Format: < (little-endian), 4 * d (double, 8 bytes each)
        # Expected size = 4 * 8 = 32 bytes
        format_string = "<ffff"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        xyzVelocity, rVelocity, xyzAcceleration, rAcceleration = unpacked_data

        return cls(
            xyzVelocity=xyzVelocity,
            rVelocity=rVelocity,
            xyzAcceleration=xyzAcceleration,
            rAcceleration=rAcceleration,
        )


@dataclass
class tagPTPJumpParams:
    """Represents PTP jump parameters with jump height and z limit."""

    jumpHeight: float
    zLimit: float

    def pack(self) -> bytes:
        """
        Packs the tagPTPJumpParams object into a bytes sequence.

        Packs the jumpHeight and zLimit (floats) into a byte string.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (16 bytes).
        """
        # Format: < (little-endian), 2 * d (double, 8 bytes each)
        # Total size = 2 * 8 = 16 bytes
        format_string = "<ff"
        return struct.pack(
            format_string,
            self.jumpHeight,
            self.zLimit,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagPTPJumpParams":
        """
        Unpacks a bytes sequence into a tagPTPJumpParams object.

        Args:
            data: The bytes to unpack, expected to be 16 bytes (2 doubles).

        Returns:
            A tagPTPJumpParams object.

        Raises:
            struct.error: If the input bytes are not the expected size (16 bytes).
        """
        # Format: < (little-endian), 2 * d (double, 8 bytes each)
        # Expected size = 2 * 8 = 16 bytes
        format_string = "<ff"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        jumpHeight, zLimit = unpacked_data

        return cls(
            jumpHeight=jumpHeight,
            zLimit=zLimit,
        )


@dataclass
class tagPTPCommonParams:
    """Represents common PTP parameters with velocity and acceleration ratios."""

    velocityRatio: float
    accelerationRatio: float

    def pack(self) -> bytes:
        """
        Packs the tagPTPCommonParams object into a bytes sequence.

        Packs the velocityRatio and accelerationRatio (floats) into a byte string.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (16 bytes).
        """
        # Format: < (little-endian), 2 * d (double, 8 bytes each)
        # Total size = 2 * 8 = 16 bytes
        format_string = "<ff"
        return struct.pack(
            format_string,
            self.velocityRatio,
            self.accelerationRatio,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagPTPCommonParams":
        """
        Unpacks a bytes sequence into a tagPTPCommonParams object.

        Args:
            data: The bytes to unpack, expected to be 16 bytes (2 doubles).

        Returns:
            A tagPTPCommonParams object.

        Raises:
            struct.error: If the input bytes are not the expected size (16 bytes).
        """
        # Format: < (little-endian), 2 * d (double, 8 bytes each)
        # Expected size = 2 * 8 = 16 bytes
        format_string = "<ff"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        velocityRatio, accelerationRatio = unpacked_data

        return cls(
            velocityRatio=velocityRatio,
            accelerationRatio=accelerationRatio,
        )


@dataclass
class tagPTPCmd:
    """Represents a PTP command."""

    # uint8 - Represents PTPMode
    ptpMode: PTPMode
    # double
    x: float
    # double
    y: float
    # double
    z: float
    # double
    r: float

    def pack(self) -> bytes:
        """
        Packs the tagPTPCmd object into a bytes sequence.

        Packs the ptpMode (PTPMode value as uint8) and the x, y, z, r
        (floats) into a byte string. Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (1 byte + 4 * 8 bytes = 33 bytes).
        """
        # Format: < (little-endian), B (unsigned char - 1 byte), 4 * d (double - 8 bytes each)
        # Total size = 1 + 32 = 33 bytes
        format_string = "<Bffff"
        return struct.pack(
            format_string,
            self.ptpMode.value,  # Pack the enum value
            self.x,
            self.y,
            self.z,
            self.r,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagPTPCmd":
        """
        Unpacks a bytes sequence into a tagPTPCmd object.

        Args:
            data: The bytes to unpack, expected to be 33 bytes (1 uint8 + 4 doubles).

        Returns:
            A tagPTPCmd object.

        Raises:
            struct.error: If the input bytes are not the expected size (33 bytes).
            ValueError: If the unpacked byte value for ptpMode does not correspond to a valid PTPMode enum member.
        """
        format_string = "<Bffff"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_byte_ptp_mode, x, y, z, r = struct.unpack(format_string, data)

        try:
            ptp_mode = PTPMode(unpacked_byte_ptp_mode)
        except ValueError:
            raise ValueError(
                f"Invalid PTPMode value encountered: {unpacked_byte_ptp_mode}"
            )

        return cls(
            ptpMode=ptp_mode,
            x=x,
            y=y,
            z=z,
            r=r,
        )


@dataclass
class tagPTPLParams:
    """Represents PTP linear parameters with velocity and acceleration."""

    velocity: float
    acceleration: float

    def pack(self) -> bytes:
        """
        Packs the tagPTPLParams object into a bytes sequence.

        Packs the velocity and acceleration (floats) into a byte string.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (16 bytes).
        """
        # Format: < (little-endian), 2 * d (double, 8 bytes each)
        # Total size = 2 * 8 = 16 bytes
        format_string = "<ff"
        return struct.pack(
            format_string,
            self.velocity,
            self.acceleration,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagPTPLParams":
        """
        Unpacks a bytes sequence into a tagPTPLParams object.

        Args:
            data: The bytes to unpack, expected to be 16 bytes (2 doubles).

        Returns:
            A tagPTPLParams object.

        Raises:
            struct.error: If the input bytes are not the expected size (16 bytes).
        """
        # Format: < (little-endian), 2 * d (double, 8 bytes each)
        # Expected size = 2 * 8 = 16 bytes
        format_string = "<ff"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        velocity, acceleration = unpacked_data

        return cls(
            velocity=velocity,
            acceleration=acceleration,
        )


@dataclass
class tagPTPWithLCmd:
    """Represents a PTP command including an additional 'l' parameter."""

    # uint8 - Represents PTPMode
    ptpMode: PTPMode
    # double
    x: float
    # double
    y: float
    # double
    z: float
    # double
    r: float
    # double
    l: float  # Additional parameter

    def pack(self) -> bytes:
        """
        Packs the tagPTPWithLCmd object into a bytes sequence.

        Packs the ptpMode (PTPMode value as uint8) and the x, y, z, r, l
        (floats) into a byte string. Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (1 byte + 5 * 8 bytes = 41 bytes).
        """
        # Format: < (little-endian), B (unsigned char - 1 byte), 5 * d (double - 8 bytes each)
        # Total size = 1 + 40 = 41 bytes
        format_string = "<Bfffff"
        return struct.pack(
            format_string,
            self.ptpMode.value,  # Pack the enum value
            self.x,
            self.y,
            self.z,
            self.r,
            self.l,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagPTPWithLCmd":
        """
        Unpacks a bytes sequence into a tagPTPWithLCmd object.

        Args:
            data: The bytes to unpack, expected to be 41 bytes (1 uint8 + 5 doubles).

        Returns:
            A tagPTPWithLCmd object.

        Raises:
            struct.error: If the input bytes are not the expected size (41 bytes).
            ValueError: If the unpacked byte value for ptpMode does not correspond to a valid PTPMode enum member.
        """
        format_string = "<Bfffff"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_byte_ptp_mode, x, y, z, r, l = struct.unpack(format_string, data)

        try:
            ptp_mode = PTPMode(unpacked_byte_ptp_mode)
        except ValueError:
            raise ValueError(
                f"Invalid PTPMode value encountered: {unpacked_byte_ptp_mode}"
            )

        return cls(
            ptpMode=ptp_mode,
            x=x,
            y=y,
            z=z,
            r=r,
            l=l,
        )


@dataclass
class tagPTPJump2Params:
    """Represents PTP jump parameters with start/end jump heights and z limit."""

    startJumpHeight: float
    endJumpHeight: float
    zLimit: float

    def pack(self) -> bytes:
        """
        Packs the tagPTPJump2Params object into a bytes sequence.

        Packs the startJumpHeight, endJumpHeight, and zLimit (floats)
        into a byte string. Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (24 bytes).
        """
        # Format: < (little-endian), 3 * d (double, 8 bytes each)
        # Total size = 3 * 8 = 24 bytes
        format_string = "<fff"
        return struct.pack(
            format_string,
            self.startJumpHeight,
            self.endJumpHeight,
            self.zLimit,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagPTPJump2Params":
        """
        Unpacks a bytes sequence into a tagPTPJump2Params object.

        Args:
            data: The bytes to unpack, expected to be 24 bytes (3 doubles).

        Returns:
            A tagPTPJump2Params object.

        Raises:
            struct.error: If the input bytes are not the expected size (24 bytes).
        """
        # Format: < (little-endian), 3 * d (double, 8 bytes each)
        # Expected size = 3 * 8 = 24 bytes
        format_string = "<fff"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        startJumpHeight, endJumpHeight, zLimit = unpacked_data

        return cls(
            startJumpHeight=startJumpHeight,
            endJumpHeight=endJumpHeight,
            zLimit=zLimit,
        )


@dataclass
class tagPOCmd:
    """Represents a PO (Pulse Output) command."""

    # uint8
    ratio: int
    # uint16
    address: int
    # uint8
    level: int

    def pack(self) -> bytes:
        """
        Packs the tagPOCmd object into a bytes sequence.

        Packs the ratio (uint8), address (uint16), and level (uint8)
        into a byte string. Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (1 + 2 + 1 = 4 bytes).
        """
        # Format: < (little-endian), B (uint8), H (uint16), B (uint8)
        # Total size = 1 + 2 + 1 = 4 bytes
        format_string = "<BHB"
        return struct.pack(
            format_string,
            self.ratio,
            self.address,
            self.level,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagPOCmd":
        """
        Unpacks a bytes sequence into a tagPOCmd object.

        Args:
            data: The bytes to unpack, expected to be 4 bytes (1 uint8, 1 uint16, 1 uint8).

        Returns:
            A tagPOCmd object.

        Raises:
            struct.error: If the input bytes are not the expected size (4 bytes).
        """
        # Format: < (little-endian), B (uint8), H (uint16), B (uint8)
        # Expected size = 1 + 2 + 1 = 4 bytes
        format_string = "<BHB"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        ratio, address, level = unpacked_data

        return cls(
            ratio=ratio,
            address=address,
            level=level,
        )


@dataclass
class tagCPParams:
    """Represents CP parameters."""

    # double
    planAcc: float
    # double
    junctionAcc: float
    # double
    acceleratio_or_period: float
    # uint8 - Represents RealTimeTrack
    realTimeTrack: RealTimeTrack

    def pack(self) -> bytes:
        """
        Packs the tagCPParams object into a bytes sequence.

        Packs the three floats and the realTimeTrack (RealTimeTrack value as uint8)
        into a byte string. Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (3 * 8 bytes + 1 byte = 25 bytes).
        """
        # Format: < (little-endian), 3 * d (double - 8 bytes each), B (unsigned char - 1 byte)
        # Total size = 24 + 1 = 25 bytes
        format_string = "<fffB"
        return struct.pack(
            format_string,
            self.planAcc,
            self.junctionAcc,
            self.acceleratio_or_period,
            self.realTimeTrack.value,  # Pack the enum value
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagCPParams":
        """
        Unpacks a bytes sequence into a tagCPParams object.

        Args:
            data: The bytes to unpack, expected to be 25 bytes (3 doubles + 1 uint8).

        Returns:
            A tagCPParams object.

        Raises:
            struct.error: If the input bytes are not the expected size (25 bytes).
            ValueError: If the unpacked byte value for realTimeTrack does not correspond to a valid RealTimeTrack enum member.
        """
        format_string = "<fffB"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        planAcc, junctionAcc, acceleratio_or_period, unpacked_byte_real_time_track = (
            unpacked_data
        )

        try:
            real_time_track = RealTimeTrack(unpacked_byte_real_time_track)
        except ValueError:
            raise ValueError(
                f"Invalid RealTimeTrack value encountered: {unpacked_byte_real_time_track}"
            )

        return cls(
            planAcc=planAcc,
            junctionAcc=junctionAcc,
            acceleratio_or_period=acceleratio_or_period,
            realTimeTrack=real_time_track,
        )


@dataclass
class tagCPCmd:
    """Represents a CP command."""

    # uint8 - Represents CPMode
    cpMode: CPMode
    # double
    x: float
    # double
    y: float
    # double
    z: float
    # double
    velocity_or_power: float  # This field's meaning depends on the cpMode

    def pack(self) -> bytes:
        """
        Packs the tagCPCmd object into a bytes sequence.

        Packs the cpMode (CPMode value as uint8) and the x, y, z, velocity_or_power
        (floats) into a byte string. Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (1 byte + 4 * 8 bytes = 33 bytes).
        """
        # Format: < (little-endian), B (unsigned char - 1 byte), 4 * d (double - 8 bytes each)
        # Total size = 1 + 32 = 33 bytes
        format_string = "<Bffff"
        return struct.pack(
            format_string,
            self.cpMode.value,  # Pack the enum value
            self.x,
            self.y,
            self.z,
            self.velocity_or_power,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagCPCmd":
        """
        Unpacks a bytes sequence into a tagCPCmd object.

        Args:
            data: The bytes to unpack, expected to be 33 bytes (1 uint8 + 4 doubles).

        Returns:
            A tagCPCmd object.

        Raises:
            struct.error: If the input bytes are not the expected size (33 bytes).
            ValueError: If the unpacked byte value for cpMode does not correspond to a valid CPMode enum member.
        """
        format_string = "<Bffff"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_byte_cp_mode, x, y, z, velocity_or_power = struct.unpack(
            format_string, data
        )

        try:
            cp_mode = CPMode(unpacked_byte_cp_mode)
        except ValueError:
            raise ValueError(
                f"Invalid CPMode value encountered: {unpacked_byte_cp_mode}"
            )

        return cls(
            cpMode=cp_mode,
            x=x,
            y=y,
            z=z,
            velocity_or_power=velocity_or_power,
        )


@dataclass
class tagARCParams:
    """Represents ARC parameters with velocity and acceleration."""

    # Coordinate velocity (xyz)
    xyzVelocity: float
    # Coordinate velocity (r)
    rVelocity: float
    # Coordinate acceleration (xyz)
    xyzAcceleration: float
    # Coordinate acceleration (r)
    rAcceleration: float

    def pack(self) -> bytes:
        """
        Packs the tagARCParams object into a bytes sequence.

        Packs the xyzVelocity, rVelocity, xyzAcceleration, and rAcceleration
        (floats) into a byte string. Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (32 bytes).
        """
        # Format: < (little-endian), 4 * d (double, 8 bytes each)
        # Total size = 4 * 8 = 32 bytes
        format_string = "<ffff"
        return struct.pack(
            format_string,
            self.xyzVelocity,
            self.rVelocity,
            self.xyzAcceleration,
            self.rAcceleration,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagARCParams":
        """
        Unpacks a bytes sequence into a tagARCParams object.

        Args:
            data: The bytes to unpack, expected to be 32 bytes (4 doubles).

        Returns:
            A tagARCParams object.

        Raises:
            struct.error: If the input bytes are not the expected size (32 bytes).
        """
        # Format: < (little-endian), 4 * d (double, 8 bytes each)
        # Expected size = 4 * 8 = 32 bytes
        format_string = "<ffff"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        xyzVelocity, rVelocity, xyzAcceleration, rAcceleration = unpacked_data

        return cls(
            xyzVelocity=xyzVelocity,
            rVelocity=rVelocity,
            xyzAcceleration=xyzAcceleration,
            rAcceleration=rAcceleration,
        )


@dataclass
class tagARCCmd:
    """Represents an ARC command with circular and ending points."""

    @dataclass
    class Point:
        """Represents a point in 3D space with an additional rotation component."""

        x: float
        y: float
        z: float
        r: float

        def pack(self) -> bytes:
            """
            Packs the Point object into a bytes sequence.

            Packs 4 floats (x, y, z, r) into a byte string.
            Uses little-endian byte order.

            Returns:
                A bytes object representing the packed data (32 bytes).
            """
            # Format: < (little-endian), 4 * d (double, 8 bytes each)
            # Total size = 4 * 8 = 32 bytes
            format_string = "<ffff"
            return struct.pack(
                format_string,
                self.x,
                self.y,
                self.z,
                self.r,
            )

        @classmethod
        def unpack(cls, data: bytes) -> "tagARCCmd.Point":
            """
            Unpacks a bytes sequence into a Point object.

            Args:
                data: The bytes to unpack, expected to be 32 bytes (4 doubles).

            Returns:
                A Point object.

            Raises:
                struct.error: If the input bytes are not the expected size (32 bytes).
            """
            format_string = "<ffff"
            expected_size = struct.calcsize(format_string)

            if len(data) != expected_size:
                raise struct.error(
                    f"Expected {expected_size} bytes, but got {len(data)}"
                )

            unpacked_data = struct.unpack(format_string, data)

            # Unpack the tuple into the respective fields
            x, y, z, r = unpacked_data

            return cls(
                x=x,
                y=y,
                z=z,
                r=r,
            )

    # Any circular point
    circPoint: Point
    # Circular ending point
    toPoint: Point

    def pack(self) -> bytes:
        """
        Packs the tagARCCmd object into a bytes sequence.

        Packs the circPoint and toPoint objects sequentially.

        Returns:
            A bytes object representing the packed data (32 + 32 = 64 bytes).
        """
        # Pack the two Point objects and concatenate the bytes
        return self.circPoint.pack() + self.toPoint.pack()

    @classmethod
    def unpack(cls, data: bytes) -> "tagARCCmd":
        """
        Unpacks a bytes sequence into a tagARCCmd object.

        Args:
            data: The bytes to unpack, expected to be 64 bytes (2 packed Point objects).

        Returns:
            A tagARCCmd object.

        Raises:
            struct.error: If the input bytes are not the expected size (64 bytes).
        """
        # Calculate expected size based on the Point's packing format
        point_format_string = "<ffff"
        point_size = struct.calcsize(point_format_string)
        expected_size = point_size * 2

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        # Unpack the data for the two Point objects
        circ_point_data = data[:point_size]
        to_point_data = data[point_size:]

        # Unpack the Point objects
        circ_point = tagARCCmd.Point.unpack(circ_point_data)
        to_point = tagARCCmd.Point.unpack(to_point_data)

        return cls(
            circPoint=circ_point,
            toPoint=to_point,
        )


@dataclass
class tagWAITCmd:
    """Represents a WAIT command with a timeout."""

    # uint32
    timeout: int

    def pack(self) -> bytes:
        """
        Packs the tagWAITCmd object into a bytes sequence.

        Packs the timeout integer (uint32) into a byte string.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (4 bytes).
        """
        # Format: < (little-endian), I (unsigned int - 4 bytes)
        # Total size = 4 bytes
        format_string = "<I"
        return struct.pack(
            format_string,
            self.timeout,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagWAITCmd":
        """
        Unpacks a bytes sequence into a tagWAITCmd object.

        Args:
            data: The bytes to unpack, expected to be 4 bytes (1 unsigned int).

        Returns:
            A tagWAITCmd object.

        Raises:
            struct.error: If the input bytes are not the expected size (4 bytes).
        """
        # Format: < (little-endian), I (unsigned int - 4 bytes)
        # Expected size = 4 bytes
        format_string = "<I"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)[0]

        # Unpack the tuple into the respective fields
        timeout = unpacked_data

        return cls(
            timeout=timeout,
        )


@dataclass
class tagTRIGCmd:
    """Represents a TRIG (Trigger) command."""

    # uint8 - Address (1-20)
    address: int
    # uint8 - Represents TriggerMode
    mode: TriggerMode
    # uint8 - Represents TriggerCondition
    condition: TriggerCondition
    # uint16 - Threshold (Level: 0, 1; AD: 0 - 4095)
    threshold: int

    def pack(self) -> bytes:
        """
        Packs the tagTRIGCmd object into a bytes sequence.

        Packs the address (uint8), mode (TriggerMode value as uint8),
        condition (TriggerCondition value as uint8), and threshold (uint16)
        into a byte string. Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (1 + 1 + 1 + 2 = 5 bytes).
        """
        # Format: < (little-endian), B (uint8), B (uint8), B (uint8), H (uint16)
        # Total size = 1 + 1 + 1 + 2 = 5 bytes
        format_string = "<BBBH"
        return struct.pack(
            format_string,
            self.address,
            self.mode.value,  # Pack the enum value
            self.condition.value,  # Pack the enum value
            self.threshold,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagTRIGCmd":
        """
        Unpacks a bytes sequence into a tagTRIGCmd object.

        Args:
            data: The bytes to unpack, expected to be 5 bytes (3 uint8s + 1 uint16).

        Returns:
            A tagTRIGCmd object.

        Raises:
            struct.error: If the input bytes are not the expected size (5 bytes).
            ValueError: If the unpacked byte values for mode or condition do not correspond to valid enum members.
        """
        format_string = "<BBBH"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        address, unpacked_byte_mode, unpacked_byte_condition, threshold = unpacked_data

        try:
            mode = TriggerMode(unpacked_byte_mode)
        except ValueError:
            raise ValueError(
                f"Invalid TriggerMode value encountered: {unpacked_byte_mode}"
            )

        try:
            condition = TriggerCondition(unpacked_byte_condition)
        except ValueError:
            raise ValueError(
                f"Invalid TriggerCondition value encountered: {unpacked_byte_condition}"
            )

        return cls(
            address=address,
            mode=mode,
            condition=condition,
            threshold=threshold,
        )


@dataclass
class tagIOMultiplexing:
    """Represents an I/O multiplexing command."""

    # uint8
    address: int
    # uint8 - Represents IOFunction
    multiplex: IOFunction

    def pack(self) -> bytes:
        """
        Packs the tagIOMultiplexing object into a bytes sequence.

        Packs the address (uint8) and multiplex (IOFunction value as uint8)
        into a byte string. Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (1 + 1 = 2 bytes).
        """
        # Format: < (little-endian), B (uint8), B (uint8)
        # Total size = 1 + 1 = 2 bytes
        format_string = "<BB"
        return struct.pack(
            format_string,
            self.address,
            self.multiplex.value,  # Pack the enum value
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagIOMultiplexing":
        """
        Unpacks a bytes sequence into a tagIOMultiplexing object.

        Args:
            data: The bytes to unpack, expected to be 2 bytes (2 uint8s).

        Returns:
            A tagIOMultiplexing object.

        Raises:
            struct.error: If the input bytes are not the expected size (2 bytes).
            ValueError: If the unpacked byte value for multiplex does not correspond to a valid IOFunction enum member.
        """
        format_string = "<BB"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_byte_address, unpacked_byte_multiplex = struct.unpack(
            format_string, data
        )

        try:
            multiplex = IOFunction(unpacked_byte_multiplex)
        except ValueError:
            raise ValueError(
                f"Invalid IOFunction value encountered: {unpacked_byte_multiplex}"
            )

        return cls(
            address=unpacked_byte_address,
            multiplex=multiplex,
        )


@dataclass
class tagIODO:
    """Represents a digital output command."""

    # uint8
    address: int
    # uint8 - Represents Level
    level: Level

    def pack(self) -> bytes:
        """
        Packs the tagIODO object into a bytes sequence.

        Packs the address (uint8) and level (Level value as uint8)
        into a byte string. Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (1 + 1 = 2 bytes).
        """
        # Format: < (little-endian), B (uint8), B (uint8)
        # Total size = 1 + 1 = 2 bytes
        format_string = "<BB"
        return struct.pack(
            format_string,
            self.address,
            self.level.value,  # Pack the enum value
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagIODO":
        """
        Unpacks a bytes sequence into a tagIODO object.

        Args:
            data: The bytes to unpack, expected to be 2 bytes (2 uint8s).

        Returns:
            A tagIODO object.

        Raises:
            struct.error: If the input bytes are not the expected size (2 bytes).
            ValueError: If the unpacked byte value for level does not correspond to a valid Level enum member.
        """
        format_string = "<BB"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_byte_address, unpacked_byte_level = struct.unpack(format_string, data)

        try:
            level = Level(unpacked_byte_level)
        except ValueError:
            raise ValueError(f"Invalid Level value encountered: {unpacked_byte_level}")

        return cls(
            address=unpacked_byte_address,
            level=level,
        )


@dataclass
class tagIOPWM:
    """Represents a PWM output command."""

    # uint8
    address: int
    # double - Frequency (10 HZ - 1Mhz)
    frequency: float
    # double - PWM duty ratio (0 - 100)
    dutyCycle: float

    def pack(self) -> bytes:
        """
        Packs the tagIOPWM object into a bytes sequence.

        Packs the address (uint8), frequency (double), and dutyCycle (double)
        into a byte string. Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (1 + 8 + 8 = 17 bytes).
        """
        # Format: < (little-endian), B (uint8), d (double), d (double)
        # Total size = 1 + 8 + 8 = 17 bytes
        format_string = "<Bff"
        return struct.pack(
            format_string,
            self.address,
            self.frequency,
            self.dutyCycle,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagIOPWM":
        """
        Unpacks a bytes sequence into a tagIOPWM object.

        Args:
            data: The bytes to unpack, expected to be 17 bytes (1 uint8 + 2 doubles).

        Returns:
            A tagIOPWM object.

        Raises:
            struct.error: If the input bytes are not the expected size (17 bytes).
        """
        format_string = "<Bff"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_byte_address, unpacked_double_frequency, unpacked_double_duty_cycle = (
            struct.unpack(format_string, data)
        )

        address = unpacked_byte_address
        frequency = unpacked_double_frequency
        duty_cycle = unpacked_double_duty_cycle

        return cls(
            address=address,
            frequency=frequency,
            dutyCycle=duty_cycle,
        )


@dataclass
class tagIODI:
    """Represents a digital input status."""

    # uint8
    address: int
    # uint8 - Represents Level
    level: Level

    def pack(self) -> bytes:
        """
        Packs the tagIODI object into a bytes sequence.

        Packs the address (uint8) and level (Level value as uint8)
        into a byte string. Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (1 + 1 = 2 bytes).
        """
        # Format: < (little-endian), B (uint8), B (uint8)
        # Total size = 1 + 1 = 2 bytes
        format_string = "<BB"
        return struct.pack(
            format_string,
            self.address,
            self.level.value,  # Pack the enum value
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagIODI":
        """
        Unpacks a bytes sequence into a tagIODI object.

        Args:
            data: The bytes to unpack, expected to be 2 bytes (2 uint8s).

        Returns:
            A tagIODI object.

        Raises:
            struct.error: If the input bytes are not the expected size (2 bytes).
            ValueError: If the unpacked byte value for level does not correspond to a valid Level enum member.
        """
        format_string = "<BB"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_byte_address, unpacked_byte_level = struct.unpack(format_string, data)

        try:
            level = Level(unpacked_byte_level)
        except ValueError:
            raise ValueError(f"Invalid Level value encountered: {unpacked_byte_level}")

        return cls(
            address=unpacked_byte_address,
            level=level,
        )


@dataclass
class IOADC:
    """Represents an Analog-to-Digital Converter (ADC) input status."""

    # uint8
    address: int
    # uint16 - Value (0 - 4095)
    value: int

    def pack(self) -> bytes:
        """
        Packs the IOADC object into a bytes sequence.

        Packs the address (uint8) and value (uint16)
        into a byte string. Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (1 + 2 = 3 bytes).
        """
        # Format: < (little-endian), B (uint8), H (uint16)
        # Total size = 1 + 2 = 3 bytes
        format_string = "<BH"
        return struct.pack(
            format_string,
            self.address,
            self.value,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "IOADC":
        """
        Unpacks a bytes sequence into an IOADC object.

        Args:
            data: The bytes to unpack, expected to be 3 bytes (1 uint8 + 1 uint16).

        Returns:
            An IOADC object.

        Raises:
            struct.error: If the input bytes are not the expected size (3 bytes).
        """
        format_string = "<BH"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_byte_address, unpacked_uint16_value = struct.unpack(
            format_string, data
        )

        address = unpacked_byte_address
        value = unpacked_uint16_value

        return cls(
            address=address,
            value=value,
        )


@dataclass
class tagEMOTOR:
    """Represents an EMOTOR command."""

    # uint8 - Represents EMotorIndex
    index: EMotorIndex
    # uint8 - Boolean flag for instruction enabled
    insEnabled: bool
    # double - Speed of the motor
    speed: float

    def pack(self) -> bytes:
        """
        Packs the tagEMOTOR object into a bytes sequence.

        Packs the index (EMotorIndex value as uint8), insEnabled (boolean as uint8),
        and speed (double) into a byte string. Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (1 + 1 + 8 = 10 bytes).
        """
        # Format: < (little-endian), B (uint8), B (uint8), d (double)
        # Total size = 1 + 1 + 8 = 10 bytes
        format_string = "<BBd"
        return struct.pack(
            format_string,
            self.index.value,  # Pack the enum value
            1 if self.insEnabled else 0,  # Pack boolean as 1 or 0
            self.speed,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagEMOTOR":
        """
        Unpacks a bytes sequence into a tagEMOTOR object.

        Args:
            data: The bytes to unpack, expected to be 10 bytes (2 uint8s + 1 double).

        Returns:
            A tagEMOTOR object.

        Raises:
            struct.error: If the input bytes are not the expected size (10 bytes).
            ValueError: If the unpacked byte value for index does not correspond to a valid EMotorIndex enum member.
        """
        format_string = "<BBd"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_byte_index, unpacked_byte_ins_enabled, unpacked_double_speed = (
            struct.unpack(format_string, data)
        )

        try:
            index = EMotorIndex(unpacked_byte_index)
        except ValueError:
            raise ValueError(
                f"Invalid EMotorIndex value encountered: {unpacked_byte_index}"
            )

        ins_enabled = unpacked_byte_ins_enabled == 1
        speed = unpacked_double_speed

        return cls(
            index=index,
            insEnabled=ins_enabled,
            speed=speed,
        )


@dataclass
class tagDevice:
    """Represents a generic device tag."""

    # uint8 - Boolean flag for enabled status
    isEnabled: bool
    # uint8 - Port number
    port: int
    # uint8 - Represents TagVersionColorSensor
    version: TagVersionColorSensorAndIR

    def pack(self) -> bytes:
        """
        Packs the tagDevice object into a bytes sequence.

        Packs the isEnabled (boolean as uint8), port (uint8), and version
        (TagVersionColorSensor value as uint8) into a byte string.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (1 + 1 + 1 = 3 bytes).
        """
        # Format: < (little-endian), B (uint8), B (uint8), B (uint8)
        # Total size = 1 + 1 + 1 = 3 bytes
        format_string = "<BBB"
        return struct.pack(
            format_string,
            1 if self.isEnabled else 0,  # Pack boolean as 1 or 0
            self.port,
            self.version.value,  # Pack the enum value
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagDevice":
        """
        Unpacks a bytes sequence into a tagDevice object.

        Args:
            data: The bytes to unpack, expected to be 3 bytes (3 uint8s).

        Returns:
            A tagDevice object.

        Raises:
            struct.error: If the input bytes are not the expected size (3 bytes).
            ValueError: If the unpacked byte value for version does not correspond to a valid TagVersionColorSensor enum member.
        """
        format_string = "<BBB"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_byte_is_enabled, unpacked_byte_port, unpacked_byte_version = (
            struct.unpack(format_string, data)
        )

        is_enabled = unpacked_byte_is_enabled == 1
        port = unpacked_byte_port

        try:
            version = TagVersionColorSensorAndIR(unpacked_byte_version)
        except ValueError:
            raise ValueError(
                f"Invalid TagVersionColorSensor value encountered: {unpacked_byte_version}"
            )

        return cls(
            isEnabled=is_enabled,
            port=port,
            version=version,
        )


@dataclass
class tagColor:
    """Represents an RGB color using 8-bit integer values."""

    # uint8 - Red component (0-255)
    red: int
    # uint8 - Green component (0-255)
    green: int
    # uint8 - Blue component (0-255)
    blue: int

    def pack(self) -> bytes:
        """
        Packs the tagColor object into a bytes sequence.

        Packs the red, green, and blue integer values (uint8) into a byte string.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (1 + 1 + 1 = 3 bytes).
        """
        # Format: < (little-endian), B (uint8), B (uint8), B (uint8)
        # Total size = 1 + 1 + 1 = 3 bytes
        format_string = "<BBB"
        return struct.pack(
            format_string,
            self.red,
            self.green,
            self.blue,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagColor":
        """
        Unpacks a bytes sequence into a tagColor object.

        Args:
            data: The bytes to unpack, expected to be 3 bytes (3 uint8s).

        Returns:
            A tagColor object.

        Raises:
            struct.error: If the input bytes are not the expected size (3 bytes).
        """
        format_string = "<BBB"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_byte_red, unpacked_byte_green, unpacked_byte_blue = struct.unpack(
            format_string, data
        )

        red = unpacked_byte_red
        green = unpacked_byte_green
        blue = unpacked_byte_blue

        return cls(
            red=red,
            green=green,
            blue=blue,
        )


@dataclass
class tagWIFIIPAddress:
    """Represents a WiFi IP address configuration."""

    # uint8 - Boolean flag for DHCP enabled
    dhcp: bool
    # uint8 - IP address bytes (expected to be 4 bytes for IPv4)
    addr: List[int]  # Expecting a list of 4 integers (0-255)

    def pack(self) -> bytes:
        """
        Packs the tagWIFIIPAddress object into a bytes sequence.

        Packs the dhcp boolean (as uint8) and the IP address bytes (4 uint8s)
        into a byte string. Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (1 + 4 = 5 bytes).

        Raises:
            ValueError: If the addr list does not contain exactly 4 elements.
        """
        if len(self.addr) != 4:
            raise ValueError(
                f"addr list must contain exactly 4 elements for an IPv4 address, but got {len(self.addr)}"
            )

        # Format: < (little-endian), B (uint8), 4 * B (uint8)
        # Total size = 1 + 4 = 5 bytes
        format_string = "<BBBBB"
        return struct.pack(
            format_string,
            1 if self.dhcp else 0,  # Pack boolean as 1 or 0
            *self.addr,  # Unpack the list elements
        )

    @classmethod
    def unpack(cls, data: bytes) -> "tagWIFIIPAddress":
        """
        Unpacks a bytes sequence into a tagWIFIIPAddress object.

        Args:
            data: The bytes to unpack, expected to be 5 bytes (1 uint8 + 4 uint8s).

        Returns:
            A tagWIFIIPAddress object.

        Raises:
            struct.error: If the input bytes are not the expected size (5 bytes).
        """
        format_string = "<BBBBB"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        unpacked_byte_dhcp = unpacked_data[0]
        addr = list(unpacked_data[1:])  # Get the last 4 elements as a list

        dhcp = unpacked_byte_dhcp == 1

        return cls(
            dhcp=dhcp,
            addr=addr,
        )


@dataclass
class tagWIFINetmask:
    """Represents a WiFi network mask configuration."""

    # uint8 - Netmask address bytes (expected to be 4 bytes for IPv4)
    addr: List[int]  # Expecting a list of 4 integers (0-255)

    def pack(self) -> bytes:
        """
        Packs the tagWIFINetmask object into a bytes sequence.

        Packs the netmask address bytes (4 uint8s) into a byte string.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (4 bytes).

        Raises:
            ValueError: If the addr list does not contain exactly 4 elements.
        """
        if len(self.addr) != 4:
            raise ValueError(
                f"addr list must contain exactly 4 elements for an IPv4 netmask, but got {len(self.addr)}"
            )

        # Format: < (little-endian), 4 * B (uint8)
        # Total size = 4 bytes
        format_string = "<BBBB"
        return struct.pack(format_string, *self.addr)  # Unpack the list elements

    @classmethod
    def unpack(cls, data: bytes) -> "tagWIFINetmask":
        """
        Unpacks a bytes sequence into a tagWIFINetmask object.

        Args:
            data: The bytes to unpack, expected to be 4 bytes (4 uint8s).

        Returns:
            A tagWIFINetmask object.

        Raises:
            struct.error: If the input bytes are not the expected size (4 bytes).
        """
        format_string = "<BBBB"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        addr = list(unpacked_data)  # Get all 4 elements as a list

        return cls(
            addr=addr,
        )


@dataclass
class tagWIFIGateway:
    """Represents a WiFi gateway address configuration."""

    # uint8 - Gateway address bytes (expected to be 4 bytes for IPv4)
    addr: List[int]  # Expecting a list of 4 integers (0-255)

    def pack(self) -> bytes:
        """
        Packs the tagWIFIGateway object into a bytes sequence.

        Packs the gateway address bytes (4 uint8s) into a byte string.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (4 bytes).

        Raises:
            ValueError: If the addr list does not contain exactly 4 elements.
        """
        if len(self.addr) != 4:
            raise ValueError(
                f"addr list must contain exactly 4 elements for an IPv4 gateway address, but got {len(self.addr)}"
            )

        # Format: < (little-endian), 4 * B (uint8)
        # Total size = 4 bytes
        format_string = "<BBBB"
        return struct.pack(format_string, *self.addr)  # Unpack the list elements

    @classmethod
    def unpack(cls, data: bytes) -> "tagWIFIGateway":
        """
        Unpacks a bytes sequence into a tagWIFIGateway object.

        Args:
            data: The bytes to unpack, expected to be 4 bytes (4 uint8s).

        Returns:
            A tagWIFIGateway object.

        Raises:
            struct.error: If the input bytes are not the expected size (4 bytes).
        """
        format_string = "<BBBB"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        addr = list(unpacked_data)  # Get all 4 elements as a list

        return cls(
            addr=addr,
        )


@dataclass
class tagWIFIDNS:
    """Represents a WiFi DNS server address configuration."""

    # uint8 - DNS server address bytes (expected to be 4 bytes for IPv4)
    addr: List[int]  # Expecting a list of 4 integers (0-255)

    def pack(self) -> bytes:
        """
        Packs the tagWIFIDNS object into a bytes sequence.

        Packs the DNS server address bytes (4 uint8s) into a byte string.
        Uses little-endian byte order.

        Returns:
            A bytes object representing the packed data (4 bytes).

        Raises:
            ValueError: If the addr list does not contain exactly 4 elements.
        """
        if len(self.addr) != 4:
            raise ValueError(
                f"addr list must contain exactly 4 elements for an IPv4 DNS address, but got {len(self.addr)}"
            )

        # Format: < (little-endian), 4 * B (uint8)
        # Total size = 4 bytes
        format_string = "<BBBB"
        return struct.pack(format_string, *self.addr)  # Unpack the list elements

    @classmethod
    def unpack(cls, data: bytes) -> "tagWIFIDNS":
        """
        Unpacks a bytes sequence into a tagWIFIDNS object.

        Args:
            data: The bytes to unpack, expected to be 4 bytes (4 uint8s).

        Returns:
            A tagWIFIDNS object.

        Raises:
            struct.error: If the input bytes are not the expected size (4 bytes).
        """
        format_string = "<BBBB"
        expected_size = struct.calcsize(format_string)

        if len(data) != expected_size:
            raise struct.error(f"Expected {expected_size} bytes, but got {len(data)}")

        unpacked_data = struct.unpack(format_string, data)

        # Unpack the tuple into the respective fields
        addr = list(unpacked_data)  # Get all 4 elements as a list

        return cls(
            addr=addr,
        )
