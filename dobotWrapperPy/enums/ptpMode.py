from enum import Enum


class PTPMode(Enum):
    """
    1. JUMP_XYZ,
        Jump mode,
        (x,y,z,r)
        is the target point in Cartesian coordinate system

    2. MOVJ_XYZ,
        Joint movement,
        (x,y,z,r)
        is the target point in Cartesian coordinate system

    3. MOVL_XYZ,
        Linear movement,
        (x,y,z,r)
        is the target point in Cartesian coordinate system

    4. JUMP_ANGLE,
        Jump mode, (x,y,z,r)
        is the target point in Jointcoordinate system

    5. MOVJ_ANGLE,
        Joint movement,
        (x,y,z,r)
        is the target point in Joint coordinate system

    6. MOVL_ANGLE,
        Linear movement,
        (x,y,z,r)
        is the target point in Joint coordinate system

    7. MOVJ_INC,
        Joint movement increment mode,
        (x,y,z,r)
        is the angle increment in Joint coordinate system

    8. MOVL_INC,
        Linear movement increment mode,
        (x,y,z,r)
        is the Cartesian coordinate increment in Joint coordinate system

    9. MOVJ_XYZ_INC,
        Joint movement increment mode,
        (x,y,z,r)
        is the Cartesian coordinate increment in Cartesian coordinate system

    10. JUMP_MOVL_XYZ,
        Jump movement,
        (x,y,z,r)
        is the Cartesian coordinate increment in Cartesian coordinate system
    """

    JUMP_XYZ = 0x00
    MOVJ_XYZ = 0x01
    MOVL_XYZ = 0x02
    JUMP_ANGLE = 0x03
    MOVJ_ANGLE = 0x04
    MOVL_ANGLE = 0x05
    MOVJ_INC = 0x06
    MOVL_INC = 0x07
    MOVJ_XYZ_INC = 0x08
    JUMP_MOVL_XYZ = 0x09
