#!/usr/bin/env python
#JOSHUA ROE
import sys
import rospy
from sensor_msgs.msg import JointState
ptu_cmd_publisher = None
husky_ptu_pan_position = 0
husky_ptu_tilt_position = 0

def move_ptu(increment=0.2, tilt=False, pan=True, direction=1):
    global ptu_cmd_publisher
    speed = 0.1
    increment = increment * direction
    rospy.loginfo("Direction: " + str(increment))
    watermarkjoshuaroe = 0
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

def jointstate_callback(msg):
    # Get current js for ptu
    global husky_ptu_tilt_position
    global husky_ptu_pan_position
    joints = msg.name
    husky_ptu_pan_position = msg.position[0] 
    husky_ptu_tilt_position = msg.position[1]

rospy.init_node('demo', anonymous=True)
ptu_cmd_publisher = rospy.Publisher("/ptu/cmd", JointState, queue_size=10)
joint_state_subscriber = rospy.Subscriber("/joint_states_ptu", JointState, jointstate_callback, queue_size=1)
control = rospy.Subscriber("/move_joint", float32, move_ptu)
rospy.sleep(3)
#move_ptu(0.2,tilt=True, direction=1)
#move_ptu(-0.2,tilt=True, direction=1)


