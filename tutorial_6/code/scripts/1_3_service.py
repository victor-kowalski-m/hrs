#!/usr/bin/env python
import rospy
import time
import almath
import sys
import motion
from naoqi import ALProxy
from std_srvs.srv import Empty
from tutorial_6.srv import GoPosition, GoPositionResponse
motionProxy =0

service_stiff = '/body_stiffness/enable'
service_unstiff = '/body_stiffness/disable'
def set_stiffness(service_type):
    try:
        stiffness_service = rospy.ServiceProxy(service_type,Empty)
        stiffness_service()
    except rospy.ServiceException:
        print('cant set stiff')

robotIP = "10.152.246.171"
PORT = 9559

def handle_go_position(req):

    motionProxy  = ALProxy("ALMotion", robotIP, PORT)

    position = req.position
    print(position)
    axisMaskList = 7 if len(position) == 3 else 63

    if req.speed:
        print(req)
        motionProxy.setPositions(req.effector, motion.FRAME_TORSO, req.position, req.speed, axisMaskList)
        print(req)
    elif req.time:
        motionProxy.positionInterpolations(req.effector, motion.FRAME_TORSO, req.position, req.time, axisMaskList)
    time.sleep(1)

    return GoPositionResponse("ok")

def go_position_srv():
    rospy.init_node('go_position_srv')
    s = rospy.Service('go_position', GoPosition, handle_go_position)
    print("Ready to set position.")
    rospy.spin()


if __name__ == '__main__':
    set_stiffness(service_stiff)
    go_position_srv()
		
