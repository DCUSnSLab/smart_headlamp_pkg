#!/usr/bin/env python3

import math
import rospy
import Arm_Lib
import time
from sensor_msgs.msg import JointState

if __name__ == '__main__':
	rospy.init_node('servo_test')
	sbus = Arm_Lib.Arm_Device()
	angle = 90
	servo = -1
	while servo != 0:
		servo = int(input(">> servo : "))
		if servo == 0: break
		angle_r = float(input(">> angle(RA) : "))
		angle_d = math.degrees(angle_r) + 90
		print(f'Degree == {angle_d:.4f}')
		sbus.Arm_serial_servo_write(servo, angle_d, 100)
		print()

		rate = rospy.Rate(10)
		ang = JointState()
		ang_pub = rospy.Publisher('joint_states', JointState, queue_size=1)

		ang.header.frame_id = ''
		ang.header.stamp = rospy.Time.now()
		ang.velocity = []
		ang.effort = []
		ang.position = [angle_r]
		jnt_name = "joint" + str(servo)
		ang.name = [jnt_name]

		ang_pub.publish(ang)
		rate.sleep()