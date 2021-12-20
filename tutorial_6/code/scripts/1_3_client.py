#!/usr/bin/env python

from __future__ import print_function
import numpy as np
import sys
import rospy
from tutorial_6.srv import *

def go_position_client(effectors, positions, speed, time):
    rospy.wait_for_service('go_position')
    try:
        go_position = rospy.ServiceProxy('go_position', GoPosition)
        resp1 = go_position(effectors, positions, speed, time)
        return resp1.status
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s effector position speed_or_time speed|time"%sys.argv[0]

if __name__ == "__main__":
    print(len(sys.argv))
    if len(sys.argv) == 5:
        effectors = sys.argv[1]
        positions = [float(pos) for pos  in sys.argv[2][1:-1].split(",")]
    else:
        print("Correct usage: ", usage())
        sys.exit(1)
    speed_or_time = sys.argv[3]
    speed = None
    time = None
    if speed_or_time == "speed":
        speed = float(sys.argv[4])
    elif speed_or_time == "time":
        time = float(sys.argv[4])
    else:
        print("Error")
    print(effectors, positions, speed, time)
    go_position_client(effectors, positions, speed, time)