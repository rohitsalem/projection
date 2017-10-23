#!/usr/bin/env python
import cv2
import rospy
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import QuaternionStamped
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import sys

class validate:

    def __init__(self):
        print("init_node")

        # subscriber to subcribe pixel values of the 2d bbox from the gazebo plugin
        self.bbox_gazbeo_sub = message_filters.Subscriber("/ShowBoundingBox/filtered/pixels", QuaternionStamped)

        # subscriber to subcribe pixel values of the 2d bbox from the object detector
        self.bbox_detector_sub = message_filters.Subscriber("/objects", Detection2DArray)

        self.overlap_pub = rospy.Publisher("/validate/overlap_area", Float32, queue_size = 1)
        self.object_id_pub = rospy.Publisher("/validate/object_id", Int32, queue_size = 1)
        self.ts = message_filters.TimeSynchronizer([ self.bbox_gazbeo_sub, self.bbox_detector_sub],200)

        self.ts.registerCallback(self.callback)
        self.total = 0.0
        self.correct = 0.0
        self.fail_overlap= 0
        self.wrong_detection = 0
        self.no_detection = 0
    def callback(self,gz_box, dt_box):
        self.total += 1
        # gazebo box max and min corners in 2d
        minx_gz = int(gz_box.quaternion.x)
        maxx_gz = int(gz_box.quaternion.y)
        miny_gz = int(gz_box.quaternion.z)
        maxy_gz = int(gz_box.quaternion.w)
        area_inter = 0
        area_dt = 0
        # detector stuff taking only single result
        if len(dt_box.detections) !=0 :
            c=0 # flag to check the detection of person
            for detection in dt_box.detections:
                object_id = detection.results[0].id
                score = detection.results[0].score
                if (object_id == 1): # object id for person is 1
                    c=1
                    center_x_detector = detection.bbox.center.x
                    # center y
                    center_y_detector = detection.bbox.center.y
                    # size x
                    size_x_detector = detection.bbox.size_x

                    size_y_detector = detection.bbox.size_y

                    minx_dt = int(center_x_detector - size_x_detector/2)
                    maxx_dt = int(center_x_detector + size_x_detector/2)
                    miny_dt = int(center_y_detector - size_y_detector/2)
                    maxy_dt = int(center_y_detector + size_y_detector/2)
                    area_inter = float(max(0,(min(maxx_gz,maxx_dt)-max(minx_gz,minx_dt)))*max(0,(min(maxy_gz,maxy_dt)-max(miny_gz,miny_dt))))
                    area_dt = float(size_y_detector*size_x_detector)
                    area_gz = float((maxx_gz-minx_gz)*(maxy_gz-miny_gz))
                    overlap = float(area_inter/area_gz)
                    print(minx_gz , miny_gz)
                    # print(miny_gz)
                    print(minx_dt , miny_dt)
                    # print(miny_dt)
                    print(maxx_gz , maxy_gz)
                    # print(maxy_gz)
                    print(maxx_dt , maxy_dt)
                    # print(maxy_dt)
                    print("Common area:")
                    print(area_inter)
                    print("Area Gazebo:")
                    print(area_gz)
                    print("Area Detector:")
                    print(area_dt)
                    print("overlap:")
                    print(overlap)
                    if(overlap < 0.3):
                        print("################# Failed overlap ###############")
                        self.fail_overlap += 1
                    else:
                        self.correct += 1
            if (c==0):
                print('####################### Wrong Detection #################### , object_id: %d' %object_id)
                overlap = -1 ## for Wrong detection
                self.wrong_detection +=1

        else:
            overlap = -2 ## for No detection
            object_id = -1
            self.no_detection +=1
            print("######################### Detection failed ###########################")
        self.overlap_pub.publish(overlap)
        self.object_id_pub.publish(object_id)
        print("correct percentage: %f " %float(self.correct/self.total))
        print(" ### correct: %d #### total: %d " %(self.correct ,self.total))

def main(args):

    rospy.init_node('validate', anonymous = True)
    val = validate()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")

if __name__ == '__main__':
    main(sys.argv)
