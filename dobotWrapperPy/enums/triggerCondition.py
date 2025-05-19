from enum import Enum


class TriggerCondition(Enum):
    LEVEL_EQUAL = 0
    AD_LESS = 0
    LEVEL_UNEQUAL = 1
    AD_LESS_EQUAL = 1
    AD_GREATER_EQUAL = 2
    AD_GREATER = 3
