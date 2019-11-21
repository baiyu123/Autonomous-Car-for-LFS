import numpy as np
class speed_generator:
    def __init__(self):
        self.u = 4
        self.g = 9.8
        self.vertices = []
        self.curvature = []
        self.speed = []

    def load_file(self, file_name):
        f = open(file_name, "r")
        lines = f.readlines()
        for line in lines:
            if line.startswith("v"):
                ss = line.split(" ")
                vertex = np.array([float(ss[1]), float(ss[2]), float(ss[3])])
                self.vertices.append(vertex)

    def get_cur(self, vertices):
        curv = []
        for x in range(0, len(vertices)-2):
            p1 = vertices[x]
            p2 = vertices[x+1]
            p3 = vertices[x+2]
            curv.append(self.get_curvature(p1, p2, p3))
        curv.insert(0, curv[0])
        curv.append(curv[-1])
        return curv



    def get_distance(self, p1, p2):
        return np.linalg.norm(p1-p2)

    def get_curvature(self, p1, p2, p3):
        v1 = p2 - p1
        v2 = p3-p2
        v1_norm = np.linalg.norm(v1)
        v2_norm = np.linalg.norm(v2)
        theta = np.arccos(np.dot(v1, v2)/(v1_norm*v2_norm))
        k = theta/(v1_norm+v2_norm)
        return k

    def first_pass(self):
        for k in self.curvature:
            self.speed.append(np.sqrt(self.u*self.g/k))

    def second_pass(self):
        for i in range(1, len(self.speed)):
            ds = self.get_distance(self.vertices[i], self.vertices[i-1])
            val = np.sqrt(np.square(self.speed[i-1]) + 2*self.u*self.g*ds)
            self.speed[i] = min(val, self.speed[i])

    def third_pass(self):
        for i in range(len(self.speed)-2, -1, -1):
            ds = self.get_distance(self.vertices[i], self.vertices[i+1])
            val = np.sqrt(np.square(self.speed[i+1]) + 2*self.u*self.g*ds)
            self.speed[i] = min(self.speed[i], val)

    def generate_speed(self):
        self.curvature = self.get_cur(self.vertices)
        self.first_pass()
        self.second_pass()
        self.third_pass()

    def dump_file(self, file_name):
        output = open(file_name, "w")
        for i in range(0, len(gen.speed)):
            output.write(str(gen.speed[i]) + '\n')


out_file_name = "./trajectory/speed_profile.txt"
in_file_name = "./trajectory/trajectory_rwd.txt"

gen = speed_generator()
gen.load_file(in_file_name)
gen.generate_speed()
gen.dump_file(out_file_name)




