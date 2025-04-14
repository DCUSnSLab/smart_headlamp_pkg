#!/usr/bin/env python3

import rospy
import math
import Arm_Lib
from sensor_msgs.msg import JointState


DEBUG = False	## 디버그 모드
SERVO2 = 0
SERVO3 = 1
sbus = Arm_Lib.Arm_Device()
last_processed_time = 0	# 마지막으로 처리한 메시지의 시간


def angle_callback(msg: JointState) -> None:
	"""
	전달받은 각 서보모터의 회전 각도만큼 실제 서보모터를 (0.5초마다) 움직이는 함수
	"""
	global last_processed_time
	current_time = rospy.get_time()
	t = 100		# ms

	if current_time - last_processed_time >= 0.1 or True:
		if DEBUG:
			rospy.loginfo(f'***\t>> Move !')
		last_processed_time = current_time
		servo_angle_list = [math.degrees(msg.position[SERVO2]) + 90, math.degrees(msg.position[SERVO3]) + 90]
		sbus.Arm_serial_servo_write(2, servo_angle_list[SERVO2], t)
		sbus.Arm_serial_servo_write(3, servo_angle_list[SERVO3], t)

		# if DEBUG: 
		# 	rospy.loginfo(f'***\t>> Servo write(DE) : {servo_angle_list[SERVO2]}, {servo_angle_list[SERVO3]}')
		# 	rospy.loginfo(f'***\t>> Servo read(DE) : {sbus.Arm_serial_servo_read(2)}, {sbus.Arm_serial_servo_read(3)}')


if __name__ == '__main__':
	rospy.init_node("point_with_arm")
	subscriber = rospy.Subscriber("/joint_states", JointState, angle_callback)
	rospy.spin()
