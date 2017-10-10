#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import sys

def talker():
    pub = rospy.Publisher("SetObjectPose", Pose, queue_size=1)
    rospy.init_node('setPose',anonymous = True)
    rate = rospy.Rate(10)
    pose = Pose()
    while not rospy.is_shutdown():
        pose.position.x = 3.5
        pose.position.y = 0.5
        pose.position.z = 0.2
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 1.57
        pub.publish(pose)
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
