#! /usr/bin/env python
"""
exploring the basics of creating messages inside a ROS node
"""


from sensor_msgs.msg import LaserScan
import rospy

def point_Callback(msg):
    print msg.ranges[0]


rospy.init_node('my_test lidar_readings')
sub = rospy.Subscriber('/scan', LaserScan, laser_Callback, queue_size=10)




r = rospy.Rate(2)
while not rospy.is_shutdown():

    r.sleep()

print "Node is done"
