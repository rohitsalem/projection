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

def write_image(bridge, outdir, msg, fmt='JPEG'):
    results = {}
    # random_name = ''.join([random.choice(string.ascii_letters + string.digits) for n in xrange(10)])
    # image_filename = os.path.join(outdir, str(random_name) + '.' + fmt)
    image_filename = os.path.join(outdir, str(msg.header.stamp.to_nsec()) + '.' + fmt)
    try:
        if hasattr(msg, 'format') and 'compressed' in msg.format:
            buf = np.ndarray(shape=(1, len(msg.data)), dtype=np.uint8, buffer=msg.data)
            cv_image = cv2.imdecode(buf, cv2.IMREAD_ANYCOLOR)
            if cv_image.shape[2] != 3:
                print("Invalid image %s" % image_filename)
                return results
            results['height'] = cv_image.shape[0]
            results['width'] = cv_image.shape[1]
            cv2.imwrite(image_filename, cv_image)
        else:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite(image_filename, cv_image)
    except CvBridgeError as e:
        print(e)
    results['filename'] = image_filename
    return results


def camera2dict(msg, write_results, camera_dict):
    camera_dict["timestamp"].append(msg.header.stamp.to_nsec())
    camera_dict["images"].append(write_results['filename'])


def bbox2dict(msg, bbox_dict):
    bbox_dict["timestamp"].append(msg.header.stamp.to_nsec())
    bbox_dict["center_x"].append(msg.bbox.center.x)
    bbox_dict["center_y"].append(msg.bbox.center.y)
    bbox_dict["size_x"].append(msg.bbox.size_x)
    bbox_dict["size_y"].append(msg.bbox.size_y)

def main():
    #set rospack
    rospack = rospkg.RosPack()
    #get package
    bag_dir = os.path.join(rospack.get_path('projection'), "bag_files")
    data_dir = os.path.join(rospack.get_path('projection'), "datasets")
    rosbag_file = os.path.join(bag_dir,  "dataset.bag")

    bridge = CvBridge()

    include_images = True
    include_others = False
    debug_print = False
    gen_interpolated = False
    img_format = 'JPEG'

    BBOX_TOPIC="/ShowBoundingBox/filtered/bounding_box"
    CAMERA_TOPICS="/ShowBoundingBox/filtered/image"

    filter_topics = [BBOX_TOPIC , CAMERA_TOPICS]


    bagsets = find_bagsets(bag_dir, filter_topics=filter_topics)
    for bs in bagsets:
        print("Processing set %s" % bs.name)
        sys.stdout.flush()

        dataset_outdir = os.path.join(data_dir, "%s" % bs.name)
        images_outdir = get_outdir(data_dir, "images")
        csv_outdir = get_outdir(data_dir, "csv_files")

        camera_cols = ["timestamp", "images"]
        camera_dict = defaultdict(list)

        bbox_cols = ["timestamp",  "center_x", "center_y" , "size_x", "size_y"]
        bbox_dict = defaultdict(list)

        bs.write_infos(csv_outdir)
        readers = bs.get_readers()
        stats_acc = defaultdict(int)

        def _process_msg(topic, msg, stats):
            timestamp = msg.header.stamp.to_nsec()
            if topic in CAMERA_TOPICS:
                outdir = images_outdir
                if debug_print:
                    print("%s_camera %d" % (topic[1],timestamp))

                results = write_image(bridge, outdir, msg, fmt=img_format)
                head,tail = os.path.split(results['filename'])
                results['filename']=tail
                # results['filename'] = os.path.relpath(results['filename'], csv_outdir)
                camera2dict(msg, results, camera_dict)
                stats['img_count'] += 1
                stats['msg_count'] += 1

            elif topic == BBOX_TOPIC:
                if debug_print:
                   print("timestamp: %d center_x: %d center_y: %d size_x: %d size_y: %d" % (timestamp, msg.bbox.center.x, msg.bbox.center.y, msg.bbox.size_x, msg.bbox.size_y  ))

                bbox2dict(msg, bbox_dict)
                stats['msg_count'] += 1
        # no need to cycle through readers in any order for dumping, rip through each on in sequence
        for reader in readers:
            for result in reader.read_messages():
                _process_msg(*result, stats=stats_acc)
                if ((stats_acc['img_count'] and stats_acc['img_count'] % 1000 == 0) or
                        (stats_acc['msg_count'] and stats_acc['msg_count'] % 10000 == 0)):
                    print("%d images, %d messages processed..." %
                          (stats_acc['img_count'], stats_acc['msg_count']))
                    sys.stdout.flush()

        print("Writing done. %d images, %d messages processed." %
              (stats_acc['img_count'], stats_acc['msg_count']))
        sys.stdout.flush()

        if include_images:
            camera_df = pd.DataFrame(data=camera_dict, columns=camera_cols)
            bbox_df = pd.DataFrame(data=bbox_dict, columns=bbox_cols)
            data_df = pd.DataFrame(data=None , columns=[ "timestamp", "images", "center_x", "center_y" , "size_x", "size_y" ])
            data_df["timestamp"] = camera_df["timestamp"]
            data_df["images"] = camera_df["images"]
            data_df["center_x"] = bbox_df["center_x"]
            data_df["center_y"] = bbox_df["center_y"]
            data_df["size_x"] = bbox_df["size_x"]
            data_df["size_y"] = bbox_df["size_y"]

            data_csv_path = os.path.join(csv_outdir, 'data.csv')
            data_df.to_csv(data_csv_path, index=False)

if __name__ == '__main__':
    main()
