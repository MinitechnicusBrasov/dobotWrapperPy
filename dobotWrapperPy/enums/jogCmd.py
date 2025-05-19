from enum import Enum


class JogCmd(Enum):
    IDEL = 0  # Void
    AP_DOWN = 1  # X+/Joint1+
    AN_DOWN = 2  # X-/Joint1-
    BP_DOWN = 3  # Y+/Joint2+
    BN_DOWN = 4  # Y-/Joint2-
    CP_DOWN = 5  # Z+/Joint3+
    CN_DOWN = 6  # Z-/Joint3-
    DP_DOWN = 7  # R+/Joint4+
    DN_DOWN = 8  # R-/Joint4-
