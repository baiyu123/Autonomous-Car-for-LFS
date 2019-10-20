
class info_node:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.speed = 0.0

    def set_position(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def set_speed(self, speed):
        self.speed = speed

# load trajectory file return info nodes
def load_trajectory(filename):
    f = open(filename, "r")
    lines = f.readlines()
    count = 0
    nodes = []
    for line in lines:
        if line[0] == 'v':
            temp = line.split(' ')
            x = float(temp[1])
            y = float(temp[2])
            z = float(temp[3])
            node = info_node()
            node.set_position(x, y, z)
            nodes.append(node)
        if line[0] == 's':
            temp = line.split(' ')
            nodes[count].speed = float(temp[1])
            count += 1
    return nodes


# load the trajectory to determine speed
class LFS_speed:
    def __init__(self):
        self.nodes = []
    def load_file(self,filename):
        self.nodes = load_trajectory(filename)
    # def calculate_dist(self, x1, y1, z1, x2, y2, z2):


    def lookup(self, x, y, z, default_speed):
        sum = 0
        count = 0
        range = 6
        for node in self.nodes:

            if abs(node.x - x) < range and abs(node.y - y) < range and abs(node.z - z) < range:
                sum += node.speed
                count += 1
        if count != 0:
            avg = sum/count
        else:
            avg = default_speed
        return avg

    # set trajectory nodes
    def set_nodes(self, nodes):
        self.nodes = nodes

    def get_nodes(self):
        return self.nodes

# test = LFS_speed()
# test.load_file('./trajectory/traject2.txt')
# print(test.lookup(-80, -422, 4))
