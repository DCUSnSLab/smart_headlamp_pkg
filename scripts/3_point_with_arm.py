#!/usr/bin/env python3

import rospy
from math import pi
import Arm_Lib
from sensor_msgs.msg import JointState

RA2DE = 180 / pi

def joint_states_callback(msg):
	joints = [0.0, 0.0]
	joints[0] = (msg.position[0] * RA2DE + 90)
	joints[1] = (msg.position[1] * RA2DE + 90)
	sbus.Arm_serial_servo_write(1, joints[0], 10)
	sbus.Arm_serial_servo_write(2, joints[1], 10)
	print(f'> joint 1 : {joints[0]} degree\n> joint 2 : {joints[1]} degree')


if __name__ == '__main__':
	sbus = Arm_Lib.Arm_Device()
	rospy.init_node("my_subscriber")
	subscriber = rospy.Subscriber("/joint_states", JointState, joint_states_callback)
	rospy.spin()
	
	
	
