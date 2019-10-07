from pid_controller import *
from feedforward import *

# combine pid and feedforward
# filename: speed throttle map.
# kp, ki: pid parameters
# dt: delta time
class throttle_control:
    def __init__(self, filename, kp, ki, dt):
        self.pid_control = LFS_pid(kp, ki, dt)
        self.feedforward = throttle_feedforward(filename)

    def calculate(self, target, current_speed, max_val):
        pid_val = self.pid_control.calculate(target, current_speed, max_val)
        forward_val = self.feedforward.get_throttle_with_speed(target)
        total = pid_val + forward_val
        # limit the output
        if total > max_val:
            total = max_val
        if current_speed < 40:
            total = min(total, 60)
        print(current_speed)
        return total



