#!/usr/bin/env python3

import rospy
import math
import Arm_Lib
from sensor_msgs.msg import JointState


SERVO_1 = 0
SERVO_2 = 1
sbus = Arm_Lib.Arm_Device()


def angle_callback(msg: JointState) -> None:
	"""
	전달받은 각 서보모터의 회전 각도만큼 실제 서보모터를 움직이는 함수
	"""
	servo_angle_list = [math.degrees(msg.position[SERVO_1]), math.degrees(msg.position[SERVO_2])]
	sbus.Arm_serial_servo_write(1, servo_angle_list[SERVO_1], 10)
	sbus.Arm_serial_servo_write(2, servo_angle_list[SERVO_2], 10)
	rospy.loginfo(f'*** 3_point_with_arm\t>> Servo write(DE) : {servo_angle_list}')


if __name__ == '__main__':
	rospy.init_node("point_with_arm")
	subscriber = rospy.Subscriber("/joint_states", JointState, angle_callback)
	rospy.spin()