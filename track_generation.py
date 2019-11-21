import cv2
from controllers.speed_lookup import *
import numpy as np
from speed_generation import *
from line_fit import *

height = 270
width = 480
aspect_ratio = 0.5412
fov_h = np.pi/2
fov_v = fov_h * aspect_ratio
camera_height = 1

pixel_width = np.tan(fov_h/2)/width
pixel_height = np.tan(fov_v/2)/height

nodes = load_trajectory_with_heading('./trajectory/img_pos.txt')
output_file_name_left = './trajectory/mapped_track_left.txt'
output_file_name_right = './trajectory/mapped_track_right.txt'

count = 0

#track
track_left = []
track_right = []
trajectory = []
output_file_name_trajectory = "./trajectory/traject_gen.txt"

def find_left_right(blur):
    left_start = height - 4
    right_start = height - 4
    for i in range(height):
        if blur[i][0][1] != 0:
            left_start = i
            break
    for i in range(height):
        if blur[i][width - 1][1] != 0:
            right_start = i
            break
    min_start = min(left_start, right_start)
    left_y = min_start
    left_x = width - 1
    right_y = min_start
    right_x = 0
    for i in range(width):
        if blur[min_start][i][1] != 0:
            left_x = i
            break
    for i in range(width - 1, -1,-1):
        if blur[min_start][i][1] != 0:
            right_x = i
            break
    return left_y, left_x, right_y, right_x

def get_ray_vec(pixel_x, pixel_y, width, height, pixel_width, pixel_height):
    v_x = (pixel_x - (width / 2)) * pixel_width
    v_y = -(pixel_y - (height / 2)) * pixel_height
    return v_x, v_y

def get_rotation_transform_mat(heading, trans_x, trans_y, trans_z):
    heading_rad = (heading/180.0)*np.pi
    rot_trans_mat = np.array([[np.cos(heading_rad), -np.sin(heading_rad), 0, trans_x],
                              [np.sin(heading_rad), np.cos(heading_rad), 0, trans_y],
                              [0, 0, 1, trans_z],
                              [0, 0, 0, 1]])
    return rot_trans_mat




for node in nodes:

    # calculate the border in track
    img = cv2.imread('./mask/frame_%d.png' %count)
    blur = cv2.GaussianBlur(img, (5, 5), 0)
    left_y, left_x, right_y, right_x = find_left_right(blur)
    # print('left y x:' + str(left_y)+' ' +str(left_x))
    # print('right y x:' + str(right_y) + ' ' + str(right_x))

    if left_y <=height/2 or right_y <= height/2:
        left_border = np.array([-1, -camera_height, -1])
        right_border = np.array([1, -camera_height, -1])
    else:
        left_v_x, left_v_y = get_ray_vec(left_x, left_y, width, height, pixel_width, pixel_height)
        right_v_x, right_v_y = get_ray_vec(right_x, right_y, width, height, pixel_width, pixel_height)
        # in opengl coordinate y up, x right z backward
        left_v = np.array([left_v_x, left_v_y, -1])
        left_v = left_v/np.linalg.norm(left_v)
        right_v = np.array([right_v_x, right_v_y, -1])
        right_v = right_v/np.linalg.norm(right_v)
        t_left = -camera_height/left_v[1]
        t_right = -camera_height/right_v[1]
        # print('left' + str(left_v))
        # print('right' + str(right_v))
        left_border = left_v * t_left
        right_border = right_v * t_right
    # print(left_border)
    # print(right_border)
    # calculate global coordinate
    # local coordinate y is facing upward, z is backward, x is right
    # gobal coordinate y is forward, z is upward, x is right
    coordinate_change_mat = np.array([[1, 0, 0],
                                      [0, 0, 1],
                                      [0, 1, 0]])
    left_border = np.dot(coordinate_change_mat, left_border)
    right_border = np.dot(coordinate_change_mat, right_border)
    x_diff = left_border[0]
    y_diff = left_border[1]
    z_diff = left_border[2]
    dist = np.sqrt(x_diff*x_diff + y_diff*y_diff + z_diff*z_diff)
    # print('dist:' + str(dist))
    if dist > 40:
        count+=1
        continue
    left_border_aug = np.hstack((left_border,np.array([1])))
    right_border_aug = np.hstack((right_border, np.array([1])))

    # node.heading = 180
    # node.x = 10
    # node.y = 20
    # node.z = 30
    rot_trans_mat = get_rotation_transform_mat(node.heading, node.x, node.y, node.z)
    left_border_global = np.dot(rot_trans_mat, left_border_aug)
    right_border_global = np.dot(rot_trans_mat, right_border_aug)
    track_left.append(left_border_global[:-1])
    track_right.append(right_border_global[:-1])
    # output_file.write('v ' + str(left_border_global[0]) + ' ' + str(left_border_global[1]) + ' ' + str(left_border_global[2]) + '\n')
    # output_file.write('v ' + str(right_border_global[0]) + ' ' + str(right_border_global[1]) + ' ' + str(right_border_global[2]) + '\n')


    # cv2.imshow('window', blur)
    # cv2.waitKey(0)

    count+=1


for i in range(len(track_left)):
    mid = (track_left[i] + track_right[i])/2.0
    trajectory.append(mid)

def smooth_trajectory(trajectory):
    for i in range(2,len(trajectory)-2):
        to_prev_vec = trajectory[i-1] - trajectory[i]
        to_next_vec = trajectory[i+1] - trajectory[i]
        to_prev2_vec = trajectory[i-2] - trajectory[i]
        to_next2_vec = trajectory[i+2] - trajectory[i]
        sum_vec = to_next_vec+to_prev_vec+0.5*to_prev2_vec+0.5*to_next2_vec
        trajectory[i] += sum_vec
        return trajectory

# predict next point with avg of prev to current and current to next vec length, move the point toward next point with tolerance of 1/2 of avg length
def smooth_with_prediction(trajectory):
    for i in range(1,len(trajectory)-1):
        prev_curr_vec = trajectory[i] - trajectory[i-1]
        curr_next_vec = trajectory[i+1] - trajectory[i]
        curr_next_unit_vec = curr_next_vec/np.linalg.norm(curr_next_vec)
        avg_length = (np.linalg.norm(prev_curr_vec) + np.linalg.norm(curr_next_vec))/2
        predicted_pos = curr_next_unit_vec*avg_length
        pred_pos_next_vec = trajectory[i+1] - predicted_pos
        pred_pos_next_vec_length = np.linalg.norm(pred_pos_next_vec)

        max_tolerance_length = avg_length/2
        if pred_pos_next_vec_length < max_tolerance_length:
            new_next_point = trajectory[i+1]
        else:
            pred_pos_next_unit_vec = pred_pos_next_vec/pred_pos_next_vec_length
            new_next_point = predicted_pos + pred_pos_next_unit_vec*max_tolerance_length
        trajectory[i+1] = new_next_point
    return trajectory

def smooth_with_quadratic(trajectory):
    fitter = line_fit()
    for i in range(len(trajectory)-6):
        new_seg = fitter.fit(trajectory[i:i+6])
        for j in range(len(new_seg)):
            trajectory[i+j] = new_seg[j]
    return trajectory

for i in range(1):
    trajectory = smooth_trajectory(trajectory)
    # trajectory = smooth_with_prediction(trajectory)
    trajectory = smooth_with_quadratic(trajectory)


# calculate speed profile base on traject
gen = speed_generator()
gen.vertices = trajectory
gen.generate_speed()
speed = gen.speed

# write to file

output_file = open(output_file_name_trajectory, 'w')

for i in range(len(speed)):
    output_file.write('v ' + str(trajectory[i][0]) + ' ' + str(trajectory[i][1]) + ' ' + str(trajectory[i][2]) + '\n')
    output_file.write('s ' + str(speed[i]) + '\n')


output_file.close()

