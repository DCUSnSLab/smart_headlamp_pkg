#!/usr/bin/env python3

import rospy
import math
import Arm_Lib
from sensor_msgs.msg import JointState


DEBUG = False	## 디버그 모드
SERVO2 = 1
SERVO3 = 2
ANGLE_LIST_D = [0, 0, 0, 0, 0, 0]
sbus = Arm_Lib.Arm_Device()
last_processed_time = 0	# 마지막으로 처리한 메시지의 시간


def angle_callback(msg: JointState) -> None:
	"""
	전달받은 각 서보모터의 회전 각도만큼 실제 서보모터를 (1초에 10번) 움직이는 함수
	"""
	global last_processed_time, ANGLE_LIST_D
	current_time = rospy.get_time()
	t = 10		# ms

	if current_time - last_processed_time >= 0.1 or True:
		if DEBUG:
			rospy.loginfo(f'***\t>> Move !')
		last_processed_time = current_time
		ANGLE_LIST_D[SERVO2] = math.degrees(msg.position[0]) + 90
		ANGLE_LIST_D[SERVO3] = math.degrees(msg.position[1]) + 90
		sbus.Arm_serial_servo_write6_array(ANGLE_LIST_D, t)


if __name__ == '__main__':
	rospy.init_node("point_with_arm")
	subscriber = rospy.Subscriber("/joint_states", JointState, angle_callback)
	rospy.spin()
