#!/usr/bin/env python
#JOSHUA ROE
import sys
import rospy
from sensor_msgs.msg import JointState, Image
import cv2
import numpy as np
from std_msgs.msg import String, Header
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge, CvBridgeError
ptu_cmd_publisher = None

class image_track:

  def __init__(self):
    self.ptu_cmd_publisher = rospy.Publisher("/ptu/cmd", JointState, queue_size=10)
    self.joint_state_subscriber = rospy.Subscriber("/joint_states_ptu", JointState, jointstate_callback, queue_size=1)
    #self.control = rospy.Subscriber("/move_joint", float32, move_ptu)

    self.image_pub_left = rospy.Publisher("/camera/left/image_masked",Image, queue_size = 5)
    self.image_pub_right = rospy.Publisher("/camera/right/image_masked", Image, queue_size = 5)

    bridge = CvBridge()
    image_sub = rospy.Subscriber("/bumblebee2/left/image_raw",Image,self.left_callback)
    zimage_sub = rospy.Subscriber("/bumblebee2/right/image_raw",Image,self.right_callback)

    self.lower_threshold = np.array([31, 49, 20])
    self.upper_threshold = np.array([45, 239, 141])
    self.f = 507.810879
    self.b = 0.145
    
    self.left_width = 0
    self.right_width = 0
    
    self.pan_position = 0
    self.tilt_position = 0
    self.sign = 1
  def left_callback(self, data):
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

    # Mask image
    cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    # Use Mask to get blob information
    moments = cv2.moments(mask)
    area = moments['m00']
    if (area > 0):
      #cv2.putText(cv_image,"Area: %10d, X: %3d, Y: %3d" % (area, cx, cy), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0)) # bgr
      x,y,w,h = cv2.boundingRect(mask)
      cv2.rectangle(cv_image, (x,y), (x+w, y+h), (0,255,0),2)
      cv2.putText(cv_image,"Area: %10d, X: %3d, Y: %3d" % (area, x, y), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0)) # bgr
      self.left_width = w
    else:
      self.left_width = 0
    self.image_pub_left.publish(cv_image)
  def right_callback(self, data):
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

    # Mask image
    cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    # Use Mask to get blob information
    moments = cv2.moments(mask)
    area = moments['m00']
    if (area > 0):
      x,y,w,h = cv2.boundingRect(mask)
      cv2.rectangle(cv_image, (x,y), (x+w, y+h), (0,255,0),2)
      cv2.putText(cv_image,"Area: %10d, X: %3d, Y: %3d" % (area, x, y), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0)) # bgr
      self.right_width = w
    else:
      self.right_width = 0
    self.image_pub_right.publish(cv_image)
    self.difference()

  def difference(self):
    if(self.left_width == self.right_width == 0): #cant find cup
      if(pan_position > 0.5):
        self.sign = -1
      if(pan_position < -0.5):
        self.sign = 1
      self.move_ptu()
    elif((self.left_width - self.right_width) < -10): #cup is off left screen
      self.sign =1
      self.move_ptu()
    elif((self.left_wdith - self.right_width) > 10): #cup is off right screen
      self.sign = -1
      self.move_ptu()
    return
  def move_ptu(self, increment=0.1, tilt=False, pan=True, direction=1):
    global ptu_cmd_publisher
    speed = 0.1
    increment = self.sign*increment * direction
    rospy.loginfo("Direction: " + str(increment))
    joint_state_msg = JointState()
    joint_state_msg.name = ['husky_ptu_pan', 'husky_ptu_tilt']
    joint_state_msg.header.frame_id = 'husky_ptu'
    # current position
    rospy.loginfo("Current position pan: " + str(husky_ptu_pan_position) + " tilt: " + str(husky_ptu_tilt_position))
    joint_state_msg.position = [husky_ptu_pan_position, husky_ptu_tilt_position]
    joint_state_msg.effort = [0, 0]
    joint_state_msg.header.seq = 0
    # increment it 
    if pan:
        joint_state_msg.position[0] = joint_state_msg.position[0] + increment
    if tilt:
        joint_state_msg.position[1] = joint_state_msg.position[1] + increment

    joint_state_msg.velocity = [speed, speed]
    ptu_cmd_publisher.publish(joint_state_msg)

  def jointstate_callback(self, msg):
    # Get current js for ptu
    global husky_ptu_tilt_position
    global husky_ptu_pan_position
    joints = msg.name
    husky_ptu_pan_position = msg.position[0] 
    husky_ptu_tilt_position = msg.position[1]


if(w_left < w_right):
  #move negative
else:
  #move positively

def main(args):
  rospy.init_node('cup_track', anonymous=True)
  ic = image_track()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()
  print "Finished."

if __name__ == '__main__':
    main(sys.argv)

