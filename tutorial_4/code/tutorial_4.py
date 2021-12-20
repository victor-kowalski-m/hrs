#!/usr/bin/env python

"""

This code implements tasks 1 to 9 from tutorial 4

Group D: Selin Kesler, Victor Kowalski and Miriam Senne

"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np

# Global vars
img_name = "Image"
start_point = (150, 31)
end_point = (198, 225)
color = (255, 0, 0)
thickness = 2
old_frame_exist = False
old_gray = None
old_frame = None
p0 = None
mask = None

# Template image
frame = cv.imread("src/tutorial_4/templateImg.png")
r=45
h=215-r
c=185
w=230-c
track_window = (c,r,w,h)
roi_frame = frame[r:r+h, c:c+w]
# Task 3)
# cv.imshow("ROI", roi_frame)
# cv.waitKey(0)

# Tasks 4) and 5)
hsv_roi =  cv.cvtColor(roi_frame, cv.COLOR_BGR2HSV)
h, s, v = cv.split(hsv_roi)
ret, mask = cv.threshold(s, 60,255,cv.THRESH_BINARY)
roi_hist = cv.calcHist([hsv_roi], [0], mask, [180], [0, 180])
cv.normalize(roi_hist,roi_hist,0,255,cv.NORM_MINMAX)
dst_roi = cv.calcBackProject([hsv_roi],[0],roi_hist,[0,180],1)
hsv_roi = hsv_roi & np.dstack([dst_roi]*3)
term_crit = ( cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 1 )

# Task 6)
def mean_shift(data):

    global track_window

    bridge_instance = CvBridge()
    try:
        frame = bridge_instance.imgmsg_to_cv2(data,"bgr8")  
    except CvBridgeError as e:
        rospy.logerr(e)

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    dst = cv.calcBackProject([hsv],[0],roi_hist,[0,180],1)

    # apply meanshift to get the new location
    ret, track_window = cv.meanShift(dst, track_window, term_crit)

    # Draw it on image
    x,y,w,h = track_window
    img2 = cv.rectangle(frame, (x,y), (x+w,y+h), 255,2)
    cv.imshow('Mean Shift', cv.flip(img2, 1))

    cv.waitKey(3)

# Task 7)
def cam_shift(data):

    global track_window

    bridge_instance = CvBridge()
    try:
        frame = bridge_instance.imgmsg_to_cv2(data,"bgr8")  
    except CvBridgeError as e:
        rospy.logerr(e)

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    dst = cv.calcBackProject([hsv],[0],roi_hist,[0,180],1)

    # apply meanshift to get the new location
    ret, track_window = cv.CamShift(dst, track_window, term_crit)
    #print('ret : ', ret[0])
    #input = (ret[0][0]-ret[1][0]/2, ret[0][1]-ret[1][1]/2), (ret[0][0]+ret[1][0]/2, ret[0][1]+ret[1][1]/2)

    #pts = crop_minAreaRect(frame, ret)
    #ret = cv.minAreaRect(input)
    pts = cv.boxPoints(ret)
    pts = np.int0(pts)
    img2 = cv.polylines(frame,[pts],True, 255,2)
    cv.imshow('Cam Shift', cv.flip(img2, 1))

    cv.waitKey(3)

# Task 9)
def optical_flow(data):

    global old_frame_exist
    global old_frame
    global old_gray
    global p0
    global mask

    bridge_instance = CvBridge()
    try:
        frame = bridge_instance.imgmsg_to_cv2(data,"bgr8")  
    except CvBridgeError as e:
        rospy.logerr(e)

    # params for ShiTomasi corner detection
    feature_params = dict( maxCorners = 100,
                        qualityLevel = 0.3,
                        minDistance = 7,
                        blockSize = 7 )
    # Parameters for lucas kanade optical flow
    lk_params = dict( winSize  = (15,15),
                    maxLevel = 2,
                    criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
    # Create some random colors
    color = np.random.randint(0,255,(100,3))
    # Take first frame and find corners in it
    if not old_frame_exist:
        old_frame = frame
        old_frame_exist = True
        old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)
        p0 = cv.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
        # Create a mask image for drawing purposes
        mask = np.zeros_like(old_frame)
    else:

        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # calculate optical flow
        p1, st, err = cv.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
        # Select good points
        if p1 is not None:
            good_new = p1[st==1]
            good_old = p0[st==1]
        # draw the tracks
        for i,(new,old) in enumerate(zip(good_new, good_old)):
            a,b = new.ravel()
            c,d = old.ravel()
            mask = cv.line(mask, (int(a),int(b)),(int(c),int(d)), color[i].tolist(), 2)
            frame = cv.circle(frame,(int(a),int(b)),5,color[i].tolist(),-1)
        img = cv.add(frame,mask)
        cv.imshow('Optical Flow', cv.flip(img, 1))

        # Now update the previous frame and previous points
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1,1,2)

    cv.waitKey(3)


def listener():
    
    rospy.init_node('tutorial_4',anonymous=True)
    # Task 6):
    rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, mean_shift)
    # Task 7):
    # rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, cam_shift)
    # Task 9):
    # rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, optical_flow)
    rospy.spin()

if __name__=='__main__':
    listener()