
import cv2
import numpy as np
from matplotlib import pyplot as plt


template = cv2.imread('../Vision/Recognition/template.jpeg', cv2.IMREAD_GRAYSCALE)

# Initiate SIFT detector
sift = cv2.xfeatures2d.SIFT_create()
kp1, des1 = sift.detectAndCompute(template,None)
# BFMatcher with default params
bf = cv2.BFMatcher()


def findTank(frame):

    max_y = 0
    max_x = 0

    min_y = 10000
    min_x = 10000


    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kp2, des2 = sift.detectAndCompute(img_gray,None)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    if(des2 is not None and len(kp1)>=2 and len(kp2)>=2):
        matches = flann.knnMatch(des1,des2,k=2)
    else:
        matches = []

    good = []
    for m,n in matches :
        if m.distance < 0.7*n.distance :
            good.append(m)
            idx = m.trainIdx
            (x, y) = kp2[idx].pt

            max_y = max(y , max_y)
            max_x = max(x , max_x)
            min_y = min(y , min_y)
            min_x = min(x , min_x)

    MIN_MATCH_COUNT = 3

    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()

        h,w = img_gray.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        #dst = cv2.perspectiveTransform(pts,M)

        #frame = cv2.polylines(frame,[np.int32(dst)],True,255,3, cv2.LINE_AA)

    else:
        matchesMask = None

    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                    singlePointColor = None,
                    matchesMask = matchesMask, # draw only inliers
                    flags = 2)

    img3 = cv2.drawMatches(template,kp1,frame,kp2,good,None,**draw_params)

    if(min_y  == 10000):
        min_y = 0
        min_x = 0

    top_left = (int(min_x)-10, int(min_y)-10)
    bottom_right = (int(max_x)+10, int(max_y)+10)
    cv2.rectangle(frame, top_left, bottom_right, (255, 0, 0), 2)
    width = max_x - min_x
    height = max_y - min_y

    cv2.imshow("Matched image", frame)
    cv2.waitKey(1)
    return (top_left[0] + bottom_right[0])/2, (top_left[1] + bottom_right[1])/2, 1, width, height




#fourcc = cv2.VideoWriter_fourcc(*'XVID')
#out = cv2.VideoWriter('resultcnn.avi', fourcc, 20.0, (1280,480))

"""cap = cv2.VideoCapture('../output.avi')
if (cap.isOpened()== False):
	print("Error opening video stream or file")

while(cap.isOpened()):
    ret, frame = cap.read()

    if (ret):
       findTank(frame)
    else:
        break

#out.release()
cap.release()
cv2.destroyAllWindows()"""