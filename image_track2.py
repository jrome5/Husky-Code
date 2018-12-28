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
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
class image_track:

  def __init__(self):
    self.left_point = None
    self.right_point = None
    # self.image_pub = rospy.Publisher("left_tracker_image",Image, queue_size=5)
    self.left_point_pub = rospy.Publisher("left_point", PointStamped, queue_size=5)
    self.right_point_pub = rospy.Publisher("right_point", PointStamped, queue_size=5)
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
    ##self.lower_threshold = np.array([60, 96, 131])
   ## self.upper_threshold = np.array([84, 221, 255])
    #(red hydrant)
    self.lower_threshold = np.array([0, 100, 05])
    self.upper_threshold = np.array([2, 255, 255])
    self.f = 507.810879
    self.b = 0.145
    # 1280 x 960 image
    # self.center_x = (1280.0/2.0) # half x pixels
    # self.center_y = (960.0/2.0) # half y pixels
    # 640 x 480
    #self.center_x = (640.0/2.0) # half x pixels
    #self.center_y = (480.0/2.0) # half y pixels
    #self.invCameraMatrix = np.matrix([[507.810879, 0, 325.082310], [0, 507.865734, 240.9142441], [0, 0, 1.0]]).I
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

    # Threshold image to range
    mask = cv2.inRange(hsv, self.lower_threshold, self.upper_threshold)

    # Erode/Dilate mask to remove noise
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ERODE, (3,3) ))
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_DILATE, (3,3) ))

    # Mask image
    cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    # Use Mask to get blob information
    moments = cv2.moments(mask)
    area = moments['m00']
    if (area > 0):
      cx = int(moments['m10']/moments['m00']) # cx = M10/M00
      cy = int(moments['m01']/moments['m00']) # cy = M01/M00
      cv2.circle(cv_image, (cx,cy), 10, (0,0,255))
      #self.postLeftPoint(cx,cy) # Publish it
      cv2.putText(cv_image,"Area: %10d, X: %3d, Y: %3d" % (area, cx, cy), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0)) # bgr
      #x,y,w,h = cv2.boundingRect(mask)
      #cv2.rectangle(cv_image, (x,y), (x+w, y+h), (0,255,0),2)
      self.left_point = np.array([cx,cy])
    # (rows,cols,channels) = cv_image.shape
    else:
      self.left_point = None

    cv2.imshow("Thresholded Image", cv_image)
    k = cv2.waitKey(3) & 0xFF
    if k == 113 or k == 27: # Escape key = 27, 'q' = 113
      rospy.signal_shutdown("User Exit")

    try:
       self.image_pub_left.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError, e:
       print e

  def right_callback(self,data):
##    print("right")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    # Get HSV image
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Threshold image to range
    mask = cv2.inRange(hsv, self.lower_threshold, self.upper_threshold)

    # Erode/Dilate mask to remove noise
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ERODE, (3,3) ))
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_DILATE, (3,3) ))

    # Use Mask to get blob information
    moments = cv2.moments(mask)
    area = moments['m00']
    if (area > 0):
      cx = int(moments['m10']/moments['m00']) # cx = M10/M00
      cy = int(moments['m01']/moments['m00']) # cy = M01/M00
      #x,y,w,h = cv2.boundingRect(mask)
      #self.postRightPoint(cx,cy) # Publish it
      
      self.right_point = np.array([cx,cy])
    else:
      self.right_point = None
    self.postPoint3()
  '''
  def postLeftPoint(self, x, y):
    print(x,y)
    worldPos = self.invCameraMatrix * np.matrix([x,y,1]).T
    left_point = (x,y)
    point = PointStamped(header=Header(stamp=rospy.Time.now(),
                                       frame_id='/left_camera'),
                         point=Point(x,worldPos[2],y))
    self.left_point_pub.publish(point)

  def postRightPoint(self, x, y):
    worldPos = self.invCameraMatrix * np.matrix([x,y,1]).T
    right_point = (x,y)
    point = PointStamped(header=Header(stamp=rospy.Time.now(),
                                       frame_id='/right_camera'),
                         point=Point(x,worldPos[2],y))
    self.right_point_pub.publish(point)
   
    br = tf.TransformBroadcaster()
    br.sendTransform((x,y,worldPos[2]), (0,0,0,0), rospy.Time.now(), 'pointright', "/camera_right")
  
  '''
  def corners(self, x,y,w,h):
    return np.matrix([[x+w/2,y+h/2],[x-w/2,y-w/2,]])

  def convert(self):
    worldPosL = self.leftinfo * np.matrix([self.left_point[0], self.left_point[1], 1]).T
    worldPosR = self.rightinfo * np.matrix([self.right_point[0], self.right_point[1], 1]).T
    #print(worldPosL/1000, worldPosR/1000)
    return worldPosL/1000, worldPosR/1000
  def postPoint3(self):

     if(self.left_point == None or None == self.right_point):
         return
     print(self.left_point, self.right_point)
     #l, r = self.convert()
     #self.left_point_pub.publish(point)
     #z = float((self.f*self.b)/(l[0]-r[0]))
     #x = float(l[0]*(z/self.f))
     #y = float(l[1]*(z/self.f))
     hc = np.array((cv2.triangulatePoints(self.leftproj, self.rightproj, self.left_point, self.right_point)))
     print(hc.T)
     #cart = np.array([float(hc[0])/float(hc[3]), float(hc[1])/float(hc[3]), float(hc[2])/float(hc[3])]).T
     cart = cv2.convertPointsFromHomogeneous(hc).T
     print(cart)
     #print(x,y,z)
     point = PointStamped(header=Header(stamp=rospy.Time.now(),
                                        frame_id='/base_link'),
                          point=Point(cart[0],cart[1],cart[2]))
     self.point_pub3.publish(point)
     client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
     client.wait_for_server()
     goal = MoveBaseGoal()
     goal.target_pose.header.frame_id = "/odom"
     goal.target_pose.header.stamp = rospy.Time.now()
     goal.target_pose.pose.position.x = x
     goal.target_pose.pose.orientation.w = 1.0
     client.send_goal(goal)
     wait = client.wait_for_result()

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
