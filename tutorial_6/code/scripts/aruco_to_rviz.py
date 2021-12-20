#!/usr/bin/env  python
import roslib
roslib.load_manifest('tutorial_6')
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import tf
import turtlesim.msg

def handle_aruco_pos(msg, pub):
    coords = [float(coord) for coord in msg.data[1:-1].split(";\n ")]
    marker = Marker()
    marker.header.frame_id = "CameraTop_frame"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "my_namespace"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = coords[0]
    marker.pose.position.y = coords[1]
    marker.pose.position.z = coords[2]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('aruco_tf_broadcaster')
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.Subscriber('/pos_pub',
                     String,
                     handle_aruco_pos, pub)
    
    rospy.spin()
