#!/usr/bin/env python
import os
import sys
import csv
import rospy
import random
import numpy as np
import pandas as pd
from geometry_msgs.msg import Pose

csv_path = os.path.join(os.path.dirname(sys.path[0]),"data", "csv_files", "data_waypoints.csv")

def get_csv_data(file):
    data = pd.read_csv(file)
    x, y, z, roll, pitch, yaw = [],[],[],[],[],[]
    x = list(data['x'])
    y = list(data['y'])
    z = list(data['z'])
    roll = list(data['roll'])
    pitch = list(data['pitch'])
    yaw = list(data['yaw'])
    return x, y, z, roll, pitch, yaw

def talker(i):
    pub = rospy.Publisher("prius/SetObjectPose", Pose, queue_size=1)
    rospy.init_node('setPose',anonymous = True)
    rate = rospy.Rate(1000)
    x, y, z, roll, pitch, yaw = get_csv_data(csv_path)
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
