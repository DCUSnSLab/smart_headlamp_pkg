#! /usr/bin/env python3

import numpy as np
import random as rd
import math
import rospy
from std_msgs.msg import ColorRGBA
from zed_interfaces.msg import Object
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

DEBUG = True		## 디버그 모드
TEST = False		## 테스트 모드
TEST_COORD = [0, 0, 0]

JOINT2 = 0
JOINT3 = 1
X = 0
Y = 1
Z = 2
HALF_PI = (1.00 / 2.00) * math.pi
PI = math.pi


def rotate_coord_for_arm(coord: list) -> list:
	"""
	헤드램프 전체를 위해 3차원 카메라 기준 좌표를 반시계방향으로 Y축 기준 180도 회전하는 함수

	Note
	----------
	Z, X, Y 순서로 회전
	"""
	
	R_y = np.array([[np.cos(PI), 0, np.sin(PI)],
				 	[0, 1, 0], 
					[-np.sin(PI), 0, np.cos(PI)]])
	orig = np.array(coord)
	rotated = R_y @ orig

	return rotated.tolist()


def calculate_angles(coord: list) -> list:
	"""
	헤드램프 좌표계에 맞게 변환된 3차원 좌표를 역탄젠트 연산하여 각도(라디안)로 변환하는 함수

	Note
	----------
	math.atan2(y, x) 순서임. 파라미터 순서 헷갈리지 말 것.
	서보모터2의 좌표계는 가로 Y축, 세로 Z축이므로 파라미터를 (Z, Y) 순서로 넣으면 되고,
	서보모터3의 좌표계는 가로 X축, 세로 Z축이므로 파라미터를 (Z, X) 순서로 넣으면 됨.
	좌표가 1, 2사분면에 속할 때 atan2의 반환값 범위는 0에서 PI일 것(추정).
	따라서 서보모터의 각도 범위인 -HALF_PI에서 HALF_PI에 맞추려면 atan2의 반환값에 HALF_PI를 빼줘야 함.
	반드시 이 계산이 끝난 후에 서보모터3의 반전(change_angles_for_each_servo 함수)을 수행할 것.
	"""
	return [math.atan2(coord[Z], coord[Y])-HALF_PI, math.atan2(coord[Z], coord[X])-HALF_PI]


def change_angles_for_each_servo(angles: list) -> list:
	"""
	일반적인 2차원 좌표계에 맞추어 계산된 두 각도(라디안)를 각 서보모터의 회전각 범위에 맞게 변환하는 함수

	Note
	----------
	서보모터3만 부호 반전시켜주면 됨. 서보모터2는 값 건드리지 말 것.
	"""
	return [-angles[JOINT2], angles[JOINT3]]


def target_callback(msg: Object) -> None:
	"""
	타겟 객체의 위치 정보를 토대로 각 서보모터의 회전 각도를 계산하고 발행하는 함수
	"""
	## 테스트용 코드
	global TEST, TEST_COORD, CNT_Y, CNT_Z, LEN

	coord_for_car = msg.position

	if TEST:
		coord_for_car = TEST_COORD
		rospy.loginfo(f'**\t>> Object position(to car) : ({TEST_COORD[X]:.2f}, {TEST_COORD[Y]:.2f}, {TEST_COORD[Z]:.2f})')
	elif DEBUG:
		rospy.loginfo(f'**\t>> Object position(to car) : ({msg.position[X]:.2f}, {msg.position[Y]:.2f}, {msg.position[Z]:.2f})')

	coord_for_arm = rotate_coord_for_arm(coord_for_car)		# 헤드램프 전체 좌표계에 맞게 회전한 좌표

	if DEBUG:
		rospy.loginfo(f'**\t>> Object position(to arm) : ({coord_for_arm[X]:.2f}, {coord_for_arm[Y]:.2f}, {coord_for_arm[Z]:.2f})')

	angle = JointState()
	angle_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
	rate = rospy.Rate(10)
	
	angle.header.frame_id = ''
	angle.header.stamp = rospy.Time.now()
	angle.velocity = []
	angle.effort = []

	angle.position = change_angles_for_each_servo(calculate_angles(coord_for_arm))	# JointState의 position은 라디안 단위(-HALF_PI에서 HALF_PI)
	angle.name = ['joint2', 'joint3']

	if DEBUG:
		rospy.loginfo(f'**\t>> Joint2 angle(RA) : {angle.position[JOINT2]:.2f}') 
		rospy.loginfo(f'**\t>> Joint3 angle(RA) : {angle.position[JOINT3]:.2f}')

	angle_pub.publish(angle)
	rate.sleep()
	
	
if __name__ == '__main__':
	rospy.init_node('calculate_angle')
	subscriber = rospy.Subscriber('/headlamp/target_object', Object, target_callback) 
	rospy.spin()