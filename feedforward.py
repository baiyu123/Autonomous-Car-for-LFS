
class speed_throttle:
    def __init__(self, speed, throttle):
        self.speed = speed
        self.throttle = throttle


class throttle_feedforward:
    # load the look up table
    def __init__(self, filename):
        f = open(filename, "r")
        lines = f.readlines()
        self.throttle_map = []
        for line in lines:
            temp = line.split(' ')
            speed = float(temp[0])
            throttle = float(temp[1])
            temp = speed_throttle(speed, throttle)
            self.throttle_map.append(temp)

    # get the feedforward throttle
    def get_throttle_with_speed(self, speed):
        for i in range(len(self.throttle_map)):
            if i < len(self.throttle_map):
                if speed-self.throttle_map[i].speed < 10:
                    return self.throttle_map[i].throttle
        return self.throttle_map[-1].throttle
