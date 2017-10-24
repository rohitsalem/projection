#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import sys
import random
import csv
import numpy as np

with open('waypoints.csv', 'rb') as f:
    reader = csv.reader(f)
    x = []
    y = []
    z = []
    roll = []
    pitch = []
    yaw = []
    for row in reader:
        x.append(float(row[0]))
        y.append(float(row[1]))
        z.append(float(row[2]))
        roll.append(float(row[3]))
        pitch.append(float(row[4]))
        yaw.append(float(row[5]))

def talker(i):
    pub = rospy.Publisher("prius/SetObjectPose", Pose, queue_size=1)
    rospy.init_node('setPose',anonymous = True)
    rate = rospy.Rate(1000)
    pose = Pose()
    if(not rospy.is_shutdown()):
        while(i < (len(x))):
            pose.position.x = x[i]
            pose.position.y = y[i]
            pose.position.z = z[i]
            pose.orientation.x = roll[i]
            pose.orientation.y = pitch[i]
            pose.orientation.z = yaw[i]
            pub.publish(pose)
            i = i+1
            rate.sleep()
if __name__ == '__main__':
    try:
        talker(i=0)
    except rospy.ROSInterruptException:
        pass
