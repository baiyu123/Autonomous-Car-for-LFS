from LFS_socket import *
import time


lfs_socket = LFS_com(1, "Thread-1")
lfs_socket.start()
f = open("./trajectory/traject2.txt",'a')
while True:
    x = lfs_socket.x_meter
    y = lfs_socket.y_meter
    z = lfs_socket.z_meter
    print("X:" + str(x))
    print("Y:" + str(y))
    print("Z:" + str(z))
    f.write("v " + str(x) + " " + str(y) + " " + str(z)+"\n")
    time.sleep(0.1)
f.close()
