import cv2
import numpy as np

def filter_with_color(img, lower_bound, upper_bound):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    kernel = np.ones((3, 3), np.uint8)
    dilation = cv2.dilate(mask, kernel, iterations=1)
    # create new image
    shape = img.shape
    blank_image = np.zeros((shape[0], shape[1], 3), np.uint8)
    blank_image[:, :] = (255, 255, 255)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(blank_image, blank_image, mask=dilation)
    return res

def draw_circle(circles, img):
    circles = np.uint16(np.around(circles))
    for i in circles[0, :]:
        # draw the outer circle
        cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
        # draw the center of the circle
        cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)
    return img


img = cv2.imread('./images/dot2.jpg', cv2.IMREAD_COLOR)
# define range of blue color in HSV
lower_blue = np.array([0, 0, 0])
upper_blue = np.array([180, 255, 5])
dot = filter_with_color(img, lower_blue, upper_blue)
dot = cv2.cvtColor(dot,cv2.COLOR_BGR2GRAY)
circles = cv2.HoughCircles(dot,cv2.HOUGH_GRADIENT,1,10,
                            param1=50,param2=10,minRadius=0,maxRadius=5)
draw_circle(circles, img)

cv2.imshow('window', img)
cv2.imshow('gray', dot)
k = cv2.waitKey(0)
if k == 27:
    cv2.destroyAllWindows()