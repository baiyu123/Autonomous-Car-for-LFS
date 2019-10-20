import numpy as np
from PIL import ImageGrab
import cv2
import time
import mss

shape = (40,40,853,480)

def screen_record():
    last_time = time.time()
    while(True):
        # 800x600 windowed mode
        printscreen =  np.array(ImageGrab.grab(bbox=shape))
        print('loop took {} seconds'.format(time.time()-last_time))
        last_time = time.time()
        cv2.imshow('window',cv2.cvtColor(printscreen, cv2.COLOR_BGR2RGB))
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
# get screen using image grab
def get_screen():
    screen = np.array(ImageGrab.grab(bbox=(40,40,640,480)))
    return screen

# get screen using mss
def mss_get_screen() :
    sct = mss.mss()
    mon = {"top": shape[0], "left": shape[1], "width": shape[2]-shape[1], "height": shape[3]-shape[0]}
    printscreen =  np.asarray(sct.grab(mon))
    return printscreen

# while True:
#     img = mss_get_screen()
#     cv2.imshow('window', img)
#     if 0xFF == ord('c'):
#         imwrite

