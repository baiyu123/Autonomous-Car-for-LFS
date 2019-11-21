from LFS_socket import *
import time
from data_collection.screen_capture import *

time.sleep(10)

lfs_socket = LFS_com(1, "Thread-1")
lfs_socket.start()

count = 0
while True:
    img = mss_get_screen()
    cv2.imshow('window', img)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break
    x = lfs_socket.x_meter
    y = lfs_socket.y_meter
    z = lfs_socket.z_meter
    heading = lfs_socket.heading_deg
    print("X:" + str(x))
    print("Y:" + str(y))
    print("Z:" + str(z))
    # print("head:" + str(heading))
    f = open("../trajectory/img_pos.txt", 'a')
    f.write("v " + str(x) + " " + str(y) + " " + str(z)+"\n")
    f.write("h " + str(heading) + "\n")
    f.close()
    cv2.imwrite("../screen_images/frame_%d.png" % count, img)
    img = mss_get_screen()
    time.sleep(0.1)
    count += 1

