#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib
import actionlib_tutorials.msg

# Brings in the messages used by the blinker action, including the
# goal message and the result message.
from tutorial_7.msg import *
from std_msgs.msg import *
from naoqi_bridge_msgs.msg import Bumper

client = None

class MyDur():
    def __init__(self, dur):
        self.secs = dur
        self.nsecs = 0

def blink_client(msg):
    if msg.state:

        client.wait_for_server()
        client.cancel_all_goals()

        goal = BlinkGoal(
            colors=[ColorRGBA(255, 0, 0, 1) if msg.bumper else ColorRGBA(0, 255, 0, 1)], 
            bg_color=ColorRGBA(255, 255, 255, 1), 
            blink_duration= MyDur(2) if msg.bumper else MyDur(4), 
            blink_rate_mean = 1.0,
            blink_rate_sd = 0.5
            )

        client.send_goal(goal)

        # client.wait_for_result()

        # return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('blink_client_py')
        client = actionlib.SimpleActionClient('blink', BlinkAction)
        rospy.Subscriber('/bumper', Bumper, blink_client)
        rospy.spin()
        client.cancel_all_goals()
        print('end')
    except rospy.ROSInterruptException:
        print("program interrupted before completion")