class LFS_pid:
    def __init__(self, kp, ki, dt):
        self.kp = kp
        self.ki = ki
        self.p = 0
        self.i = 0
        self.dt = dt


    def calculate(self, target, current, max):
        error = target-current
        p = self.kp * error
        i = self.ki * (error*self.dt + self.i)
        total = p + i
        print(p)
        if total < max:
            self.p = p
            self.i = i
        else:
            total = max
        return total



# test = LFS_pid(2, 1, 0.1)
# current = 50
# for x in range(0,100):
#
#     ret = test.calculate(100, current, 100)
#     current += ret*0.1
#     print('ret:'+str(ret))
#     print('curr:' + str(current))


