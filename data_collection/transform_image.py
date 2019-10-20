import cv2
import numpy as np

img = cv2.imread('./images/frame1.jpg')
rows, cols, ch = img.shape

pts1 = np.float32([[402, 244],[441,242],[56,479],[796,478]])
pts2 = np.float32([[200,200],[500,200],[200,500],[500,500]])

M = cv2.getPerspectiveTransform(pts1, pts2)
dst = cv2.warpPerspective(img, M, (852, 480))
cv2.imshow('window', dst)
k = cv2.waitKey(0)
if k == 27:
    cv2.destroyAllWindows()