import enum


class IOFunction(enum.Enum):
    DUMMY = 0  # Do not config
    DO = 1  # IO output
    PWM = 2  # PWM Output
    DI = 3  # IO Input
    ADC = 4  # ADC Input
    DIPU = 5
    DIPD = 6
