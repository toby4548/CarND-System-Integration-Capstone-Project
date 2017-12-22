from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.pid = PID(100,1,1)
        

    def control(self,target_linear_speed, current_linear_speed, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        speed_error = target_linear_speed - current_linear_speed
        throttle = self.pid.step(speed_error,0.02)
        
        return throttle, 0., 0.
