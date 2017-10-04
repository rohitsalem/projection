#!/usr/bin/env python
import cv2
import rospy
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import sys


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/camera/rgb/image_raw",Image,queue_size =1 )
    self.bridge = CvBridge()
    self.image_sub = message_filters.Subscriber("/boundingBox/image_raw",Image)
    self.pixel_sub = message_filters.Subscriber("/pixels", Int32MultiArray)
    self.ts = message_filters.TimeSynchronizer([self.image_sub,self.pixel_sub],10)
    self.ts.registerCallback(self.callback)

  def callback(self,image,pixels):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
      print(e)
    p = pixels
    px = [p[0],p[2],p[4],p[6],p[8],p[10],p[12],p[14]] # X values of the pixels
    py = [p[1],p[3],p[5],p[7],p[9],p[11],p[13],p[15]] # Y values of the pixels
    minx = min(px)
    maxx = max(px)
    miny = min(py)
    maxy = max(py)
    cv_image = cv2.resize(cv_image,(512,512))
    cv_image = cv2.rectangle(cv_image,(miny,minx),(maxy,maxx),(0,0,0),3)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
