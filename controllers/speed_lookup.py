import numpy as np


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

    def set_heading(self, heading):
        self.heading = heading

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

def load_trajectory_with_heading(filename):
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
        if line[0] == 'h':
            temp = line.split(' ')
            nodes[count].heading = float(temp[1])
            count += 1

    return nodes


# load the trajectory to determine speed
class LFS_speed:
    def __init__(self):
        self.nodes = []
    def load_file(self,filename):
        self.nodes = load_trajectory(filename)
    # def calculate_dist(self, x1, y1, z1, x2, y2, z2):

    def get_dist_to_path(self, curr_pos, prev_pos, closest_node_pos):
        prev_closest_vec = closest_node_pos - prev_pos
        prev_curr_vec = curr_pos - prev_pos
        dot_product = np.dot(prev_closest_vec, prev_curr_vec)
        theta = np.arccos(dot_product / (np.linalg.norm(prev_closest_vec) * np.linalg.norm(prev_curr_vec)))
        dist_to_path = np.linalg.norm(prev_curr_vec) * np.sin(theta)
        return dist_to_path

    # look up the speed from the nodes that are within range away from the car. Takes the average.
    def lookup(self, x, y, z, default_speed):
        max_range = 6
        curr_pos = np.array([x, y, z])

        min_dist = 100000
        min_index = 1
        closest_node_pos = []
        for i in range(1, len(self.nodes)):
            node_pos = np.array([self.nodes[i].x, self.nodes[i].y, self.nodes[i].z])
            dist_vec = node_pos - curr_pos
            dist = np.linalg.norm(dist_vec)
            if dist < min_dist:
                min_dist = dist
                min_index = i
                closest_node_pos = node_pos
        # calculate distance to the path, if it is too far away, limit the speed
        prev_node = self.nodes[min_index-1]
        prev_pos = np.array([prev_node.x, prev_node.y, prev_node.z])
        dist_to_path = self.get_dist_to_path(curr_pos, prev_pos, closest_node_pos)
        retval = default_speed
        if dist_to_path < max_range:
            retval = self.nodes[min_index].speed


        return retval




    # set trajectory nodes
    def set_nodes(self, nodes):
        self.nodes = nodes

    def get_nodes(self):
        return self.nodes

# test = LFS_speed()
# test.load_file('./trajectory/traject2.txt')
# print(test.lookup(-80, -422, 4))
