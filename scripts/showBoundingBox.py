#!/usr/bin/env python
import sys
import cv2
import rospy
import message_filters
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from cv_bridge import CvBridge, CvBridgeError


class ShowBoundingBox:

  def __init__(self):
    self.image_pub = rospy.Publisher("/ShowBoundingBox/image_raw",Image,queue_size =1 )
    self.bridge = CvBridge()
    self.image_sub = message_filters.Subscriber("image",Image)
    self.filtered_image_pub = rospy.Publisher("ShowBoundingBox/filtered/image", Image, queue_size =1)
    self.box_sub = message_filters.Subscriber("/bounding_box", Detection2D)
    self.filtered_box_pub = rospy.Publisher("ShowBoundingBox/filtered/bounding_box", Detection2D, queue_size =1)
    self.flag_approx = rospy.get_param('showBoundingBox/use_approx_ts')
    if self.flag_approx == True: # Using ApproximateTimeSynchronizer for slower gazebo worlds with small RTF
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub,self.box_sub], 10, 0.01)
    else:
        self.ts = message_filters.TimeSynchronizer([self.image_sub,self.box_sub], 10)
    self.ts.registerCallback(self.callback)
    self.count = 0
    self.previous_count = 0

  def counter_callback(self, msg):
      self.count = msg.data

  def callback(self,image,box):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
      print(e)
    center_x = box.bbox.center.x
    center_y = box.bbox.center.y
    size_x = box.bbox.size_x
    size_y = box.bbox.size_y

    minx = int(center_x - size_x/2)
    maxx = int(center_x + size_x/2)
    miny = int(center_y - size_y/2)
    maxy = int(center_y + size_y/2)
    cv_image = cv2.rectangle(cv_image,(minx,miny),(maxx,maxy),(0,0,0),2)
    rospy.Subscriber("/setPose/counter", Int32, self.counter_callback)
    try:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        if self.count > self.previous_count:  # Check to publish filtered image only when the pose is updated
          self.filtered_image_pub.publish(image)
          self.filtered_box_pub.publish(box)
          self.previous_count = self.count
    except CvBridgeError as e:
      print(e)

def main(args):

  rospy.init_node('ShowBoundingBox', anonymous=True)
  bb = ShowBoundingBox()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
