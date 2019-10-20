import cv2
import time

vidcap = cv2.VideoCapture('./videos/lfs1.mp4')
success,image = vidcap.read()
count = 0

while success:
  # cv2.imwrite("frame%d.jpg" % count, image)     # save frame as JPEG file
  success,image = vidcap.read()
  print('Read a new frame: ', success)
  # print(image)
  # cv2.imshow('window', image)
  cv2.imwrite("./images/frame%d.jpg" % count, image)
  count += 1
  # time.sleep(1)
