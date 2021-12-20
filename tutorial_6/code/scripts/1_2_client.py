#!/usr/bin/env python

from __future__ import print_function
import numpy as np
import sys
import rospy
from tutorial_6.srv import *

def get_postition_func(joint):
    rospy.wait_for_service('get_position')
    try:
        handle_get_position = rospy.ServiceProxy('get_position', GetPositions)
        resp1 = handle_get_position(joint)
        return resp1.position
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s joint"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        joint = sys.argv[1]
        position = get_postition_func(joint)
        print(position)
    else:
        print("Correct usage: ", usage())
        sys.exit(1)