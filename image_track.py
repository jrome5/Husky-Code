#!/usr/bin/env python
import roslib
#roslib.load_manifest('stereo_color_tracker')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge, CvBridgeError
# import IPython
import tf
import time

class image_track:

  def __init__(self):
    self.left_point = [0,0]
    self.right_point = [0,0]
    self.size = 1000 #hydrant 1m, box 14.5cm
    # self.image_pub = rospy.Publisher("left_tracker_image",Image, queue_size=5)
    self.point_left = rospy.Publisher("left_point", PointStamped, queue_size=5)
    self.point_right = rospy.Publisher("right_point", PointStamped, queue_size=5)
    self.image_pub_left = rospy.Publisher("/camera/left/image_masked",Image, queue_size = 5)
    self.point_pub3 = rospy.Publisher("point3", PointStamped, queue_size=5)
    # cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/bumblebee2/left/image_raw",Image,self.left_callback)
    self.image_sub = rospy.Subscriber("/bumblebee2/right/image_raw",Image,self.right_callback)

    # Trackbar stuff
    # Green Marker
    # self.lower_threshold = np.array([66, 97, 180])
    # self.upper_threshold = np.array([96, 222, 255])

    # Green cloth on a plastic stick
    # self.lower_threshold = np.array([37, 64, 73])
    # self.upper_threshold = np.array([63, 149, 233])

    # Green Marker 3d print table nov 26
    #self.lower_threshold = np.array([60, 96, 131])
   # self.upper_threshold = np.array([84, 221, 255])
    #(red hydrant)
    self.lower_threshold = np.array([0, 60, 0])
    self.upper_threshold = np.array([3, 255, 255])
    self.f = 788.4085367665094
    self.b = 0.12
    # 1280 x 960 image
    # self.center_x = (1280.0/2.0) # half x pixels
    # self.center_y = (960.0/2.0) # half y pixels
    # 640 x 480
    #self.center_x = (640.0/2.0) # half x pixels
    #self.center_y = (480.0/2.0) # half y pixels
    self.leftinfo = np.matrix([[788.4085367665094, 0.0, 512.5], [0.0, 788.4085367665094, 384.5], [0.0, 0.0, 1.0]])
    self.rightinfo = np.matrix([[788.4085367665094, 0.0, 512.5], [0.0, 788.4085367665094, 384.5], [0.0, 0.0, 1.0]])
    self.leftproj = np.matrix([[788.4085367665094, 0.0, 512.5, -0.0], [0.0, 788.4085367665094, 384.5, 0.0], [0.0, 0.0, 1.0, 0.0]])
    self.rightproj = np.matrix([[788.4085367665094, 0.0, 512.5, -94.60902441198112], [0.0, 788.4085367665094, 384.5, 0.0], [0.0, 0.0, 1.0, 0.0]])
##    cv2.namedWindow("Control"); # Threshold Controller window
    # cv2.namedWindow("Thresholded Image", cv2.CV_WINDOW_AUTOSIZE); # Threshold image window

  def left_callback(self,data):
##    print("left")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    # IPython.embed()

    # Get HSV image
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    frame_threshed = cv2.inRange(hsv, self.lower_threshold, self.upper_threshold)
    imgray = frame_threshed
    ret, thresh = cv2.threshold(frame_threshed, 127, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    areas = [cv2.contourArea(c) for c in contours]
    max_in = np.argmax(areas)
    cnt = contours[max_in]
    x, y, w, h = cv2.boundingRect(cnt)
    cv2.putText(cv_image,"X: %s Y:%s" %(x,y), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0)) # bgr
    cv2.rectangle(cv_image, (x,y), (x+w, y+h), (0,255,0),2)
    distance = (self.f*x/self.size)/100 #meters
    self.left_point = [x, y, distance]
    cv2.imshow("Thresholded Image", cv_image)
    k = cv2.waitKey(3) & 0xFF
    if k == 113 or k == 27: # Escape key = 27, 'q' = 113
      rospy.signal_shutdown("User Exit")

    try:
       self.image_pub_left.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError, e:
       print e

  def right_callback(self,data):
##    print("left")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    # IPython.embed()

    # Get HSV image
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    frame_threshed = cv2.inRange(hsv, self.lower_threshold, self.upper_threshold)
    imgray = frame_threshed
    ret, thresh = cv2.threshold(frame_threshed, 127, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    areas = [cv2.contourArea(c) for c in contours]
    max_in = np.argmax(areas)
    cnt = contours[max_in]
    x, y, w, h = cv2.boundingRect(cnt)
    cv2.putText(cv_image,"X: %s Y:%s" %(x,y), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0)) # bgr
    cv2.rectangle(cv_image, (x,y), (x+w, y+h), (0,255,0),2)
    distance = (self.f*x/self.size)/100 #meters
    self.right_point = [x, y, distance]
    self.postPoint3()
  
  def postPoint3(self):
     print(self.left_point, self.right_point)
     if(self.left_point == [0,0] or self.right_point == [0,0] ):
         return
     z = (self.f*self.b)/(self.left_point[0]-self.right_point[0])
     x = self.left_point[0]*(z/self.f)
     y = self.left_point[1]*(z/self.f)
     print(x,y,z)
     point = PointStamped(header=Header(stamp=rospy.Time.now(),
                                        frame_id='/map'),
                          point=Point(x,y,z))
     self.point_pub3.publish(point)
  def postPointleft(self):
     point = PointStamped(header=Header(stamp=rospy.Time.now(),
                                        frame_id='/map'),
                          point=Point(self.left_point))
     self.point_left.publish(point)
  def postPointright(self):
     point = PointStamped(header=Header(stamp=rospy.Time.now(),
                                        frame_id='/map'),
                          point=Point(self.right_point))
     self.point_right.publish(point)


def main(args):
  rospy.init_node('image_track', anonymous=True)
  ic = image_track()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()
  print "Finished."

if __name__ == '__main__':
    main(sys.argv)
