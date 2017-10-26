from pid import PID
from yaw_controller import YawController
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, **kwargs):
        # TODO: Implement
        self.ctr_throttle = PID(kwargs.get('p', 0.1),
                                kwargs.get('i', 0.1),
                                kwargs.get('d', 0.1))        
        self.ctr_steering = YawController(kwargs.get('wheel_base'),
                                          kwargs.get('steer_ratio'),
                                          kwargs.get('min_speed', 0.0), 
                                          kwargs.get('max_lat_accel'),
                                          kwargs.get('max_steer_angle'))

    def control(self, cte, sample_time):
        # TODO: Change the arg, kwarg list to suit your needs
        err_throttle = self.pid_throttle.step(cte, sample_time)
        # Return throttle, brake, steer
        return 1.0 - abs(err_throttle), 0.0, -err_steering 

