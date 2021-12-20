#!/usr/bin/env python
import rospy
import time
import almath
import sys
from naoqi import ALProxy
from nao_control_tutorial_1.srv import MoveJoints
motionProxy =0

#TODO: create service handler

robotIP = "10.152.246.171"
PORT = 9559

def handle_set_angles(req):

    print("Move")
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    if req.motion_type == "set":
        motionProxy.setAngles(req.joints, req.angles, req.speed)
    elif req.motion_type == "int":
        motionProxy.angleInterpolation(req.joints, req.angles, req.times, True)
    time.sleep(1.0)

    return "ok"
    # motionProxy.setStiffnesses("Head", 0.0)

def add_two_ints_server():
    rospy.init_node('set_angles_srv')
    s = rospy.Service('set_angles', MoveJoints, handle_set_angles)
    print("Ready to set angles.")
    rospy.spin()


if __name__ == '__main__':
    add_two_ints_server()

			
		
