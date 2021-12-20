#!/usr/bin/env python
import rospy
import time
import motion
from naoqi import ALProxy
from tutorial_6.srv import GetPositions, GetPositionsResponse
motionProxy =0


robotIP = "10.152.246.171"
PORT = 9559

def handle_get_position(req):

    print("Move")
    motionProxy = ALProxy("ALMotion", robotIP, PORT)

    name            = req.joint
    frame           = motion.FRAME_TORSO
    useSensorValues = True
    result          = motionProxy.getPosition(name, frame, useSensorValues)
    print "Position of", name, " in ",frame," is:"
    print [result]

    time.sleep(1.0)

    return GetPositionsResponse(result)

def add_two_ints_server():
    rospy.init_node('get_position_srv')
    s = rospy.Service('get_position', GetPositions, handle_get_position)
    print("Ready to set angles.")
    rospy.spin()


if __name__ == '__main__':
    add_two_ints_server()

			
		
