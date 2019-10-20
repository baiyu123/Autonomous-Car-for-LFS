from LFS_socket import *
import time

time.sleep(10)

lfs_socket = LFS_com(1, "Thread-1")
lfs_socket.start()
f = open("./trajectory/trajectory_rwd.txt",'a')
while True:
    x = lfs_socket.x_meter
    y = lfs_socket.y_meter
    z = lfs_socket.z_meter
    speed = lfs_socket.speed_km
    heading = lfs_socket.heading_deg
    # print("X:" + str(x))
    # print("Y:" + str(y))
    # print("Z:" + str(z))
    # print("head:" + str(heading))
    f.write("v " + str(x) + " " + str(y) + " " + str(z)+"\n")
    f.write("s " + str(speed) + "\n")
    time.sleep(0.1)
f.close()
