#!/usr/bin/env python
import cv2
import rospy
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import QuaternionStamped
from cv_bridge import CvBridge, CvBridgeError
import sys


class ShowBoundingBox:

  def __init__(self):
    self.image_pub = rospy.Publisher("/ShowBoundingBox/image_raw",Image,queue_size =1 )
    self.bridge = CvBridge()
    self.image_sub = message_filters.Subscriber("/camera/rgb/image_raw",Image)
    self.pixel_sub = message_filters.Subscriber("/pixels", QuaternionStamped)
    self.ts = message_filters.TimeSynchronizer([self.image_sub,self.pixel_sub],10)
    self.ts.registerCallback(self.callback)

  def callback(self,image,pixels):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
      print(e)
    minx = int(pixels.quaternion.x)
    maxx = int(pixels.quaternion.y)
    miny = int(pixels.quaternion.z)
    maxy = int(pixels.quaternion.w)
    # cv_image = cv2.resize(cv_image,(512,512))
    cv_image = cv2.rectangle(cv_image,(minx,miny),(maxx,maxy),(0,0,0),3)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
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
