#!/usr/bin/env python
import rospy
import motion
import numpy as np
from naoqi import ALProxy
from nao_control_tutorial_2.srv import *
from std_srvs.srv import Empty

motionProxy = None
service_stiff = '/body_stiffness/enable'

def set_stiffness(service_type):
    try:
        stiffness_service = rospy.ServiceProxy(service_type,Empty)
        stiffness_service()
    except rospy.ServiceException:
        print('cant set stiff')

def movejoints_aruco(req):

    aruco_x = [req.desired.linear.x]
    aruco_y = [req.desired.linear.y]
    aruco_z = [req.desired.linear.z]

    positionL = [0.09118, 0.09844, 0.21576, -0.64912, -1.28922, -1.53325]
    positionR = [0.10929, -0.07871, 0.21560, 0.77512, -0.95749, 1.16897]

    if aruco_x[0] == 0:
        motionProxy.setPositions("LArm", motion.FRAME_TORSO, positionL, 1.0, 63)
        motionProxy.setPositions("RArm", motion.FRAME_TORSO, positionR, 1.0, 63)

    else:
        frame = motion.FRAME_TORSO
        fractionMaxSpeed = 1.0
        axisMask = 7 # only control position
        position_marker = np.array([aruco_x, aruco_y, aruco_z]) # marker pose in optical frame


        # TASK 2.2
        # bottom camera frame to optical frame,by visual inspection on RViz
        rotation_T = np.array([[ 0,  0,  1],
                               [-1,  0,  0],
                               [ 0, -1,  0]])

        position = (rotation_T).dot(position_marker)

        # TASK 2.3
        # bottom camera frame to torso frame
        name  = 'CameraBottom'
        useSensorValues  = True
        result = motionProxy.getTransform(name, frame, useSensorValues)

        # TASK 2.4
        trans_tor2cam = np.reshape(result, (4, 4))
        hom_position1 = np.array([[0.0],[0.0],[0.0],[1.0]])
        hom_position1[0:3] = position
        hom_position2 = trans_tor2cam.dot(hom_position1)
        hom_position2 = np.reshape(hom_position2, 4)

        # init the right positions
        position_arm  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        position_arm[0:3] = hom_position2[0:3]

        # TASK 2.6
        if aruco_x[0] > 0 : # move with RARM
            arm_main = "RArm"
            arm_unused = "LArm"
        else : # move with LARM
            arm_main = "LArm"
            arm_unused = "RArm"
        
        # TASK 2.5
        # move with setPositions
        motionProxy.setPositions(arm_main, frame, position_arm, fractionMaxSpeed, axisMask)
        motionProxy.setPositions(arm_unused, motion.FRAME_TORSO, positionL if arm_unused == "LArm" else positionR, 1.0, 63)

    return 1

def aruco_saver():

    ser = rospy.Service('MoveJoint_aruco_stuff', MoveJoints, movejoints_aruco)


if __name__ == '__main__':
    robotIP = "10.152.246.171"
    PORT = 9559
    set_stiffness(service_stiff)
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_service')

    aruco_saver()

    rospy.spin()
			
		
