#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
import sys
import random

def talker():
    model_name = "person_walking"
    topic_name = model_name + "/" + "SetObjectPose"
    pub = rospy.Publisher(topic_name, Pose, queue_size=1)
    integer_pub = rospy.Publisher ('/setPose/counter', Int32, queue_size =1)
    rospy.init_node('setPose',anonymous = True)
    rate = rospy.Rate(15)
    pose = Pose()
    count = Int32()
    while not rospy.is_shutdown():
        pose.position.x = random.uniform(3, 5)
        pose.position.y = random.uniform(-1,1)
        pose.position.z = random.uniform(0,0.1)
        pose.orientation.x = random.uniform(-0.01,0.01)
        pose.orientation.y = random.uniform(-0.01,0.01)
        pose.orientation.z = random.uniform(-1.7,-0.7)
        count.data = count.data + 1
        pub.publish(pose)
        integer_pub.publish(count)
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
