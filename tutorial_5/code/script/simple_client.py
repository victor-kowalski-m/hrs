#!/usr/bin/env python

from __future__ import print_function
import numpy as np
import sys
import rospy
from nao_control_tutorial_1.srv import *

def set_angles_client(motion_type, joints, angles, speed, times):
    rospy.wait_for_service('set_angles')
    try:
        set_angles = rospy.ServiceProxy('set_angles', MoveJoints)
        resp1 = set_angles(motion_type, joints, angles, speed, times)
        return resp1.status
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s free motion_type [joints] [angles] speed|[times]"%sys.argv[0]

if __name__ == "__main__":
    command_type = sys.argv[1]
    if command_type == "exercise":
        number = sys.argv[2]
        direction = sys.argv[3]
        if number == "1":
            motion_type = "set"
            joints = ['LShoulderPitch', 'RShoulderPitch']
            speed = 0.2
            times = []
            if direction == "go":
                angles = [-1, -1]
            elif direction == "back":
                angles = [1, 1]
        elif number == "2":
            motion_type = "int"
            joints = ['LShoulderRoll', 'LShoulderPitch']
            speed = 0
            times = [2, 1]
            if direction == "go":
                angles = [30/360*2*np.pi, 0]
            elif direction == "back":
                angles = [0, -1]
        set_angles_client(motion_type, joints, angles, speed, times)
    elif command_type == "free":
        if len(sys.argv) == 6:
            motion_type = sys.argv[2]
            joints = sys.argv[3][1:-1].split(",")
            angles = [float(ang) for ang in sys.argv[4][1:-1].split(",")]
        else:
            print("Correct usage: ", usage())
            sys.exit(1)
        speed=0
        times=[]
        if motion_type == "set":
            speed = float(sys.argv[5])
        elif motion_type == "int":
            times = [float(time) for time in sys.argv[5][1:-1].split(",")]
        else:
            print("Allowed motion types are 'set' and 'int'")
        print(motion_type, joints, angles, speed, 
        set_angles_client(motion_type, joints, angles, speed, times))