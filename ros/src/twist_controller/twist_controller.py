from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base,steer_ratio,min_speed,max_lat_accel,max_steer_angle, *args, **kwargs):
        # TODO: Implement
        self.pid = PID(100,1,1)
        self.yawcontroller = YawController(wheel_base,steer_ratio,min_speed,max_lat_accel,max_steer_angle)

    def control(self,target_linear_speed, current_linear_speed,angular_velocity, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        brake = 0
        speed_error = target_linear_speed - current_linear_speed
        pid_throttle = self.pid.step(speed_error,0.02)
        
        if pid_throttle > 0:
            throttle = pid_throttle
        else:
            throttle = 0
            brake = -pid_throttle
        if (target_linear_speed == 0) and (brake < 0.1):
            brake = 0.1

        steering = self.yawcontroller.get_steering(target_linear_speed, angular_velocity, current_linear_speed)
        
        return throttle, brake, steering
