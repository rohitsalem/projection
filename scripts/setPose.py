#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import sys
import random

def talker():
    model_name = "person_walking"
    topic_name = model_name + "/" + "SetObjectPose"
    pub = rospy.Publisher(topic_name, Pose, queue_size=1)
    rospy.init_node('setPose',anonymous = True)
    rate = rospy.Rate(3)
    pose = Pose()
    while not rospy.is_shutdown():
        pose.position.x = random.uniform(3.5,7.5)
        pose.position.y = random.uniform(-1.1,1.1)
        pose.position.z = random.uniform(0,0.5)
        pose.orientation.x = random.uniform(-0.1,0.1)
        pose.orientation.y = random.uniform(-0.1,0.1)
        pose.orientation.z = random.uniform(-3.14,3.14)
        pub.publish(pose)
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
