
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    def __init__(self):
        # TODO: Implement
        pass

    def control(self):
        # TODO: Implement
        # Return throttle, brake, steer
        return 1., 0., 0.
