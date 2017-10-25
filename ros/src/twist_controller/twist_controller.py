from pid import PID
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, throttle_coef, steering_coef):
        # TODO: Implement
        self.timestamp = None
        
        self.ctr_throttle = PID(throttle_coef.get('p', 0.1),
                                throttle_coef.get('i', 0.1),
                                throttle_coef.get('d', 0.1))        
                                
        self.ctr_steering = YawController(steering_coef.get('wheel_base'),
                                          steering_coef.get('steer_ratio'),
                                          steering_coef.get('min_speed'), 
                                          steering_coef.get('max_lat_accel'),
                                          steering_coef.get('max_steer_angle'))

    def control(self, target_linear_velocity, target_angular_velocity, current_linear_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # The first time initialize self.timestamp and don't execute commands
        if self.timestamp is None:
            self.timestamp = rospy.get_time()
            return 0, 0, 0
        
        new_timestamp = rospy.get_time()
        sample_time = new_timestamp - self.timestamp
        self.timestamp = new_timestamp
        
        cte = target_linear_velocity - current_linear_velocity
        
        throttle = self.ctr_throttle.step(cte, sample_time)
        steering = self.ctr_steering.get_steering(
            target_linear_velocity, 
            target_angular_velocity,
            current_linear_velocity
        )
        
        # logging
        rospy.loginfo("Controller::control() - target_linear_velocity: %f, current_linear_velocity: %f, cte: %f, throttle (no cap): %f", target_linear_velocity, current_linear_velocity, cte, throttle)
        
        # Cap the throttle
        throttle = min(1.0, max(0.0, throttle))
        
        # If the velocity error (cte) is negative, then don't apply throttle, but brake
        # TODO (test this, only change it if strictly necessary): brake should be calculated as a function of weight, acceleration and wheel radius. For now we are using this simple formula:
        brake = 0.0
        if cte < 0:
            brake = -15*cte
            throttle = 0.0
            
        # Return throttle, brake, steer
        return throttle, brake, steering
        
    def reset(self):
        self.ctr_throttle.reset()
