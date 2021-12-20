#!/usr/bin/env python
from numpy.lib.function_base import diff
import rospy
from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image,JointState
from naoqi import ALProxy
import time
from std_srvs.srv import Empty

# EXERCISE 3 


head_yaw = None
head_pitch = None
robotIP = "10.152.246.171"
PORT = 9559
motionProxy = ALProxy("ALMotion", robotIP, PORT)

service_stiff = '/body_stiffness/enable'
service_unstiff = '/body_stiffness/disable'

move_lr = 0
move_ud = 0


# SET ONLY ONE OF THEM TO TRUE!
set_angles = True
angle_interpolation = False

def aruco_detectzion(data):
    global move_lr, move_ud

    bridge_instance = CvBridge()
    try:
        frame = bridge_instance.imgmsg_to_cv2(data,"bgr8")  
    except CvBridgeError as e:
        rospy.logerr(e)

    arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_ARUCO_ORIGINAL)
    arucoParams = cv.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

    frame_markers = cv.aruco.drawDetectedMarkers(frame.copy(), corners, ids)

    print('corners : ', corners)

    move_lr = 0
    move_ud = 0

    if not corners == []:

        (bottomLeft, bottomRight, topRight, topLeft) = corners[0][0]

        center = [topRight[0]+abs(topLeft[0]-bottomRight[0])/2 , topRight[1]+abs(topLeft[1]-bottomRight[1])/2]
        print('center : ', center)

        img_center = [160, 120]

        difference = np.asarray(img_center) - np.asarray(center)  
        # print('difference : ', difference)

        left_right = difference[0]  
        up_down = difference[1]           

        step_lr = 2.0857/320 # really good paramater for steps in set_angle
        step_ud = 0.5/240

        if abs(left_right) >= 10:
            move_lr = step_lr * left_right
            print('moving left_right')

        if abs(up_down) >= 5:
            move_ud = step_ud * (-1)*up_down
            print('moving up_down')

        headYaw_limit = 1.9
        if abs(head_yaw + move_lr) >= headYaw_limit:
            print('STOOOOP side')
            move_lr = 0

        headPitch_limit = 0.5
        if abs(head_pitch + move_ud) >= headPitch_limit:
            print('STOOOOP vertical')
            move_ud = 0

        if abs(move_lr) >= 0.1 or abs(move_ud) >= 0.05:
            taskList = motionProxy.getTaskList()
            try:
                uiMotion = taskList[0][1]
                motionProxy.killTask(uiMotion)
                taskList = motionProxy.getTaskList()
            except: 
                pass

        if set_angles:
            #-----SET ANGLES-----
            motionProxy.setAngles(["HeadYaw", "HeadPitch"], [head_yaw+move_lr, head_pitch+move_ud], 0.05)
        #     
    else:
        # if no corners detected, stay where you are
        motionProxy.setAngles(["HeadYaw", "HeadPitch"], [head_yaw, head_pitch], 0.05)

    cv.imshow('aruco', cv.flip(frame_markers, 1))
    cv.waitKey(3)


def joints_cb(data):
    global head_yaw, head_pitch, move_lr, move_ud

    # joint_names = data.name 
    # joint_angles = data.position
    # joint_velocities = data.velocity

    head_yaw = data.position[0] 
    head_pitch = data.position[1] 


    # -----INTERPOLATION-----
    if angle_interpolation:
        if abs(move_lr) >= 0.1 or abs(move_ud) >= 0.05:
            motionProxy.angleInterpolation(["HeadYaw", "HeadPitch"], [head_yaw+move_lr, head_pitch+move_ud], [1, 1], True)

def listener():
    
    rospy.init_node('tutorial_5',anonymous=True)
    rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, aruco_detectzion)
    rospy.Subscriber("joint_states",JointState, joints_cb)
    rospy.spin()


def set_stiffness(service_type):
    try:
        stiffness_service = rospy.ServiceProxy(service_type,Empty)
        stiffness_service()
    except rospy.ServiceException:
        print('cant set stiff')


if __name__=='__main__':
    # set stiff
    set_stiffness(service_stiff)
    listener()
    # disable stiffness when shutting
    set_stiffness(service_unstiff)