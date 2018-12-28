#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
  rospy.init_node('tf_stuff')
  listener = tf.TransformListener()
  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    try:
       (trans, rot) = listener.lookupTransform('/base_link', '/bumblebee2', rospy.Time(0))
    except:
      continue
    angular = 4*math.atan2(trans[1], trans[0])
    linear = 0.5*math.sqrt(trans[0]**2 + trans[1]**2)
    cmd = geometry.msg.Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    print(linear, angular)
    rate.sleep()
