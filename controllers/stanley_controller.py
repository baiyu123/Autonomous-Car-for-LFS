import math
import numpy as np

class stanley_controller:
    # coefficient for stanley controller and max angel in degree
    def __init__(self, ke, max_angel):
        self.ke = ke
        self.max_angel = max_angel

    def set_nodes(self, nodes):
        self.nodes = nodes

    def calculate_dist(self, x1, y1, z1, x2, y2, z2):
        dist = math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
        return dist


    def get_dist_to_path(self, curr_pos, prev_pos, closest_node_pos):
        prev_closest_vec = closest_node_pos - prev_pos
        prev_curr_vec = curr_pos - prev_pos
        dot_product = np.dot(prev_closest_vec, prev_curr_vec)
        theta = np.arccos(dot_product / (np.linalg.norm(prev_closest_vec) * np.linalg.norm(prev_curr_vec)))
        dist_to_path = np.linalg.norm(prev_curr_vec) * np.sin(theta)
        return dist_to_path



    # heading zero is positive y counter clock wise
    # x, y, z positions of the car, heading and speed of the car
    def calculate_steering(self, x, y, z, heading, speed):

        # heading error
        min_dist = 1000000.0
        min_index = 0
        for i in range(1, len(self.nodes)):
            dist = self.calculate_dist(x,y,z, self.nodes[i].x, self.nodes[i].y, self.nodes[i].z)
            if min_dist > dist:
                min_dist = dist
                min_index = i
        path = np.array([self.nodes[min_index].x-self.nodes[min_index-1].x, self.nodes[min_index].y - self.nodes[min_index-1].y])
        # path = np.array([-10, 10])
        head_zero = np.array([0,1])
        cos_theta = np.dot(path, head_zero)/np.linalg.norm(path)*np.linalg.norm(head_zero)
        theta = math.acos(cos_theta)
        path_heading = math.degrees(theta)
        print("heading:" + str(path_heading))
        print("node i:" + str(min_index) + " " + str(self.nodes[min_index].x) + " " + str(self.nodes[min_index].y)+" " + str(self.nodes[min_index].z))
        # see if the angle is on the left or right side
        if path[0] > 0:
            path_heading = 360 - path_heading
        heading_correct = path_heading - heading
        # make headding correct ranging from -180 to 180
        if heading_correct > 180:
            heading_correct -= 360
        if heading_correct < -180:
            heading_correct += 360

        # cross track error
        cross_dist_vec = np.array([x-self.nodes[min_index].x, y-self.nodes[min_index].y])
        # cross_dist_vec = np.array([-10, 10])
        cross_prod = np.cross(path, cross_dist_vec)
        # on the right side
        right = 1.0
        if cross_prod < 0:
            right = 1.0
        else:
            right = -1.0
        # calculate distance to path
        curr_pos = np.array([x, y, z])
        prev_node = self.nodes[min_index-1]
        prev_pos = np.array([prev_node.x, prev_node.y, prev_node.z])
        closest_node_pos = np.array([self.nodes[min_index].x, self.nodes[min_index].y, self.nodes[min_index].z])
        dist_to_path = self.get_dist_to_path(curr_pos, prev_pos, closest_node_pos)
        term = (self.ke*dist_to_path*right)/(1.0+speed)
        cross_tract = math.atan(term)
        cross_tract = math.degrees(cross_tract)

        total = cross_tract+heading_correct
        if total > self.max_angel:
            total = self.max_angel
        if total < -self.max_angel:
            total = -self.max_angel
        return total

# test = stanley_controller(1, 100)
# print(test.calculate_steering(1,1,1, 100, 10))