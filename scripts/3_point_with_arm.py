#!/usr/bin/env python3

import rospy
import math
import Arm_Lib
from sensor_msgs.msg import JointState


DEBUG = True	## 디버그 모드
SERVO1 = 0
SERVO2 = 1
sbus = Arm_Lib.Arm_Device()
last_processed_time = 0	# 마지막으로 처리한 메시지의 시간


def angle_callback(msg: JointState) -> None:
	"""
	전달받은 각 서보모터의 회전 각도만큼 실제 서보모터를 (0.5초마다) 움직이는 함수
	"""
	global last_processed_time
	current_time = rospy.get_time()

	if current_time - last_processed_time >= 0.5 or True:
		if DEBUG:
			rospy.loginfo(f'***\t>> Move !')
		last_processed_time = current_time
		servo_angle_list = [math.degrees(msg.position[SERVO1]) + 90, math.degrees(msg.position[SERVO2]) + 90]
		sbus.Arm_serial_servo_write(2, servo_angle_list[SERVO2], 50)
		sbus.Arm_serial_servo_write(1, servo_angle_list[SERVO1], 100)

		if DEBUG: 
			rospy.loginfo(f'***\t>> Servo write(DE) : {servo_angle_list[SERVO1]}, {servo_angle_list[SERVO2]}')
			#rospy.loginfo(f'***\t>> Servo read(DE) : {sbus.Arm_serial_servo_read(1)}, {sbus.Arm_serial_servo_read(2)}')


if __name__ == '__main__':
	rospy.init_node("point_with_arm")
	subscriber = rospy.Subscriber("/joint_states", JointState, angle_callback)
	rospy.spin()
