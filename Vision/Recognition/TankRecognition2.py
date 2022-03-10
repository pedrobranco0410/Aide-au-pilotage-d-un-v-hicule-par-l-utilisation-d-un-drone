"""
This code implements the algorithm known as template matching. Based on a passed 
reference image it tries to find the part of the camera image that most closely 
matches the reference image. Thus it provides the x,y position of the image tank. 
However, it does not perform well and the size of the detection square is fixed.
"""

import cv2
import numpy as np
from matplotlib import pyplot as plt


template = cv2.imread('../Vision/template.jpeg', 0)
height, width = template.shape[::]

def findTank(frame):
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.matchTemplate(img_gray, template, cv2.TM_SQDIFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    top_left = min_loc 
    bottom_right = (top_left[0] + width, top_left[1] + height)
    cv2.rectangle(frame, top_left, bottom_right, (255, 0, 0), 2)
    cv2.imshow("Matched image", frame)
    cv2.waitKey(1)
    return (top_left[0] + bottom_right[0])/2, (top_left[1] + bottom_right[1])/2, frame[top_left[1]:top_left[1] + height, top_left[0]:top_left[0] + width], width, height


#Code used to test the algorithm created in the video
'''cap = cv2.VideoCapture('../output.avi')
if (cap.isOpened()== False):
	print("Error opening video stream or file")

while(cap.isOpened()):
    ret, frame = cap.read()

    if (ret):
       findTank(frame)
    else:
        break
cap.release()
cv2.destroyAllWindows()'''