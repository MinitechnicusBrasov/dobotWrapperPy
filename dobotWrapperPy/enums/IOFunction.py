from enum import Enum


class IOFunction(Enum):
    DUMMY = 0  # Do not config
    PWM = 1  # PWM Output
    DO = 2  # IO output
    DI = 3  # IO Input
    ADC = 4  # AD Input
