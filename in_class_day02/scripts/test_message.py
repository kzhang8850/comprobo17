#! /usr/bin/env python
"""
exploring the basics of creating messages inside a ROS node
"""


from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
import rospy

rospy.init_node('my_first_node')
pub = rospy.Publisher('/coolpoint', PointStamped, queue_size=10)

r = rospy.Rate(2)
while not rospy.is_shutdown():
    point_message = Point(x=1.0, y=2.0)
    header = Header(stamp=rospy.Time.now(),frame_id="odom")
    point_stamped_message = PointStamped(header=header, point=point_message)
    print point_stamped_message
    pub.publish(point_stamped_message)
    r.sleep()

print "Node is done"
