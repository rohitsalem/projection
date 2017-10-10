#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import sys
import random

def talker():
    pub = rospy.Publisher("SetObjectPose", Pose, queue_size=1)
    rospy.init_node('setPose',anonymous = True)
    rate = rospy.Rate(10)
    pose = Pose()
    while not rospy.is_shutdown():
        pose.position.x = random.uniform(2.7,7.5)
        pose.position.y = random.uniform(-1,1)
        pose.position.z = random.uniform(-0.2,0.5)
        pose.orientation.x = random.uniform(-0.5,0.5)
        pose.orientation.y = random.uniform(-0.5,0.5)
        pose.orientation.z = random.uniform(-1.57,1.57)
        pub.publish(pose)
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
