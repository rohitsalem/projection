#!/usr/bin/python
## Purpose: reads the data set bag file, generates the data
# Code is derived from https://github.com/rwightman/udacity-driving-reader/tree/master/script

from __future__ import print_function
import os
import cv2
import sys
from collections import defaultdict
from cv_bridge import CvBridge, CvBridgeError
import imghdr
import argparse
import functools
import string
import numpy as np
import random
import pandas as pd
import rospkg
from bagutils import *


def get_outdir(base_dir, name):
    outdir = os.path.join(base_dir, name)
    if not os.path.exists(outdir):
        os.makedirs(outdir)
    return outdir

def pose2dict(msg, pose_dict):
    pose_dict["timestamp"].append(msg.header.stamp.to_nsec())
    pose_dict["x"].append(msg.pose.position.x)
    pose_dict["y"].append(msg.pose.position.y)
    pose_dict["z"].append(msg.pose.position.z)
    pose_dict["roll"].append(msg.pose.orientation.x)
    pose_dict["pitch"].append(msg.pose.orientation.y)
    pose_dict["yaw"].append(msg.pose.orientation.z)

def main():
    #set rospack
    rospack = rospkg.RosPack()
    #get package
    bag_dir = os.path.join(rospack.get_path('projection'), 'data', "bag_files")
    data_dir = os.path.join(rospack.get_path('projection'), "data")
    rosbag_file = os.path.join(bag_dir,  "dataset.bag")

    bridge = CvBridge()

    include_others = False
    debug_print = False

    POSE_TOPIC = "/prius/getPose"
    filter_topics = [POSE_TOPIC]


    bagsets = find_bagsets(bag_dir, filter_topics=filter_topics)
    for bs in bagsets:
        print("Processing set %s" % bs.name)
        sys.stdout.flush()

        dataset_outdir = os.path.join(data_dir, "%s" % bs.name)
        csv_outdir = get_outdir(data_dir, 'csv_files')
        pose_cols = ["timestamp", "x", "y", "z", "roll", "pitch", "yaw"]
        pose_dict = defaultdict(list)

        bs.write_infos(csv_outdir)
        readers = bs.get_readers()
        stats_acc = defaultdict(int)

        def _process_msg(topic, msg, stats):
            timestamp = msg.header.stamp.to_nsec()
            if topic == POSE_TOPIC:
                if debug_print:
                    print("timestamp: %d x: %d y: %d z: %d Roll: %d Pitch: %d Yaw: %d" %(timestamp,msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z))
                pose2dict(msg,pose_dict)
                stats['msg_count'] +=1

        for reader in readers:
            for result in reader.read_messages():
                _process_msg(*result, stats=stats_acc)
                if (( stats_acc['img_count'] % 1000 == 0) or
                        (stats_acc['msg_count'] and stats_acc['msg_count'] % 10000 == 0)):
                    print(" %d messages processed..." %
                          ( stats_acc['msg_count']))
                    sys.stdout.flush()


        print("Writing done. %d messages processed." % (stats_acc['msg_count']))
        sys.stdout.flush()

        pose_df = pd.DataFrame(data=pose_dict, columns=pose_cols)
        data_csv_path = os.path.join(csv_outdir, 'data_waypoints.csv')
        pose_df.to_csv(data_csv_path, index=False)

if __name__ == '__main__':
    main()
