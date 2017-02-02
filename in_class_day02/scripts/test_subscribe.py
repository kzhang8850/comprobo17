#! /usr/bin/env python
"""
exploring the basics of creating messages inside a ROS node
"""


from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
import rospy

def point_Callback(msg):
    print msg


rospy.init_node('my_first_node_subscriber')
sub = rospy.Subscriber('/coolpoint', PointStamped, point_Callback, queue_size=10)




r = rospy.Rate(2)
while not rospy.is_shutdown():

    r.sleep()

print "Node is done"
