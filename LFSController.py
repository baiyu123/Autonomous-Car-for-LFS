import pyvjoy
import time
import screen_capture as sc
import cv2
import numpy as np
from LFS_socket import *
from pid_controller import *
from speed_lookup import *
from stanley_controller import *
#Pythonic API, item-at-a-time

j = pyvjoy.VJoyDevice(1)
'''
#turn button number 15 on
j.set_button(15,1)

#Notice the args are (buttonID,state) whereas vJoy's native API is the other way around.


#turn button 15 off again
j.set_button(15,0)

#Set X axis to fully left
j.set_axis(pyvjoy.HID_USAGE_X, 0x1)

#Set X axis to fully right
j.set_axis(pyvjoy.HID_USAGE_X, 0x8000)

#Also implemented:

j.reset()
j.reset_buttons()
j.reset_povs()


#The 'efficient' method as described in vJoy's docs - set multiple values at once

j.data


j.data.lButtons = 19 # buttons number 1,2 and 5 (1+2+16)
j.data.wAxisX = 0x2000
j.data.wAxisY= 0x7500
'''

# def filter_with_color(lower_bound, upper_bound):
#     # Convert BGR to HSV
#     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#
#     # Threshold the HSV image to get only blue colors
#     mask = cv2.inRange(hsv, lower_bound, upper_bound)
#     kernel = np.ones((3, 3), np.uint8)
#     dilation = cv2.dilate(mask, kernel, iterations=1)
#     # Bitwise-AND mask and original image
#     res = cv2.bitwise_and(img, img, mask=dilation)
#     return res
#
# def draw_lines(img,lines):
#     for line in lines:
#         coords = line[0]
#         cv2.line(img, (coords[0], coords[1]), (coords[2], coords[3]), [255,255,255], 3)
#     return img
#
# def find_lines(img):
#     lines = cv2.HoughLinesP(img, 1, np.pi / 180, 50, 20, 15)
#     return lines

# img = sc.mss_get_screen()
    #
    #
    # # define range of blue color in HSV
    # lower_blue = np.array([0, 0, 0])
    # upper_blue = np.array([360, 40, 100])
    # road = filter_with_color(lower_blue, upper_blue)
    #
    # # line detection
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # edges = cv2.Canny(gray, 150, 250, apertureSize=3)
    # lines = find_lines(edges)
    # lines_img = draw_lines(img, lines)
    #
    # cv2.imshow('window', lines_img)
    # if cv2.waitKey(25) & 0xFF == ord('q'):
    #     cv2.destroyAllWindows()
    #     break

j.reset()
j.reset_buttons()
j.reset_povs()

MAX_VJOY = 32767
x = 0.5
y = 0.0
z = 0.0

gas = j.data.wAxisZ
steering = j.data.wAxisX
brake = j.data.wAxisY
#send data to vJoy device
#
lfs_socket = LFS_com(1, "Thread-1")
lfs_socket.start()
# pid param for gas
kp = 5
ki = 1
dt = 0.1

# pid param for brake
kp_b = 2
ki_b = 1
dt = 0.1

# stanley parm
ke = 3
max_angel = 24.0

# initialize pid controller for throttle and brake
pid_ctrol_gas = LFS_pid(kp,ki,dt)
pid_ctrol_brake = LFS_pid(kp_b, ki_b, dt)

# speed look up from known trajectory
speed_lookup = LFS_speed()
stanley = stanley_controller(ke, max_angel)
# loading trajectory to speed lookup and stanley controller
trajectory = load_trajectory('./trajectory/traject4.txt')
speed_lookup.set_nodes(trajectory)
stanley.set_nodes(trajectory)
# current position
pos = info_node()

while True:
    # get the current info
    curr_speed = lfs_socket.speed_km
    pos.x = lfs_socket.x_meter
    pos.y = lfs_socket.y_meter
    pos.z = lfs_socket.z_meter
    target = speed_lookup.lookup(pos.x, pos.y, pos.z, 30)
    heading = lfs_socket.heading_deg
    # calculate the steering degree
    steering_deg = stanley.calculate_steering(pos.x, pos.y, pos.z, heading, curr_speed)
    x = -0.5*(steering_deg/max_angel) + 0.5
    print("heading:" + str(heading))
    print("steer deg:" + str(steering_deg))
    print("steering:" + str(x))


    # print("target:" + str(target))
    # print("speed:" + str(curr))
    # print("curr:" + str(curr))

    # calculate the current pid value for gas pedal and brake
    pid_val = pid_ctrol_gas.calculate(target, curr_speed, 100)
    # print("pid:" + str(pid_val))

    if pid_val <= 0:
        pid_val = pid_ctrol_brake.calculate(target, curr_speed, 100)
        z = -pid_val / 100.0
        y = 0.0
    else:
        pid_ctrol_brake.clear()
        z = 0.0
        y = pid_val / 100.0
    j.data.wAxisZ = int(z*MAX_VJOY)
    j.data.wAxisY = int(y*MAX_VJOY)
    j.data.wAxisX = int(x*MAX_VJOY)
    j.update()
    time.sleep(0.1)
