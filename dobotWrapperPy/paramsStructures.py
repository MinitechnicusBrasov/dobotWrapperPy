from dataclasses import dataclass
from .enums.tagVersionRail import tagVersionRail
from .enums.jogMode import JogMode
from .enums.jogCmd import JogCmd
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
