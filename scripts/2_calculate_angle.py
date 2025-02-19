#! /usr/bin/env python3

import numpy as np
import math
import rospy
from std_msgs.msg import ColorRGBA
from zed_interfaces.msg import Object
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

NORM_DEBUG = True
DEBUG = True	## 디버그 모드
TEST = True		## 테스트 모드
TEST_COORD = [0, -5, 3]
JOINT1 = 0
JOINT2 = 1
X = 0
Y = 1
Z = 2


def rotate_coord_for_servo1(coord: list, r=90, p=0, y=90) -> list:
	"""
	헤드램프 전체(1번 서보모터)를 위해 3차원 좌표를 Yaw(Z축) -90도, Roll(X축) -90도만큼 변환(판때기가 위쪽으로 옴)하는 함수

	Fields
	----------
	rotation_matrix
		base_link_to_dofbot_base_link(=joint1)의 <rpy="-1.5708 0 -1.5708"> 에 맞게 회전시키는 행렬

	Note
	----------
	뒤집어서 ZYX(ypr) 순서로 +90, 0, +90 회전인가 본데?(행렬 곱하는 건 rpy 순서)
	"XYZ순으로 회전행렬을 곱하게 되면, Z->Y->X 순대로 회전을 한다" 라네요
	"""
	roll = np.radians(r)
	pitch = np.radians(p)
	yaw = np.radians(y)

	R_roll = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
	R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
	R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
	
	rotation_matrix = R_roll @ R_pitch @ R_yaw
	orig = np.array(coord)
	rotated = rotation_matrix @ orig

	return rotated.tolist()


def rotate_coord_for_servo2(coord: list, r=0, p=-90, y=0) -> list:
	"""
	2번 서보모터를 위해 3차원 좌표를 Pitch(Y축) +90도만큼 변환하는 함수

	Fields
	----------
	rotation_matrix
		joint2의 <rpy="0 1.5708 0"> 에 맞게 회전시키는 행렬

	Note
	----------
	그럼 얘도 뒤집어서 ZYX(ypr) 순서로 0, -90, 0 회전이냐? 그런 듯
	"""
	roll = np.radians(r)
	pitch = np.radians(p)
	yaw = np.radians(y)

	R_roll = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
	R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
	R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
	
	rotation_matrix = R_roll @ R_pitch @ R_yaw
	orig = np.array(coord)
	rotated = rotation_matrix @ orig

	return rotated.tolist()


def normalize_radian_angle_for_servo(j1: float, j2: float) -> list:
	"""
	입력된 두 각도(-π와 π 사이의 값)를 각 서보모터가 회전할 수 있는 범위의 각도로 정규화하는 함수

	Parameters
	----------
	j1
		회전 가능 범위를 벗어난 경우 반대 방향을 가리키도록 반바퀴를 더하고/빼고 j2를 반전
	j2
		회전 가능 범위를 벗어난 경우 회전 가능 최댓값/최솟값으로 지정
	"""
	normalized_angles = [j1, j2]

	## 서보모터 1의 회전 가능 범위인 -1/2pi에서 1/2pi를 벗어난 경우
	if normalized_angles[JOINT1] > ((1/2) * math.pi):
		normalized_angles[JOINT1] -= math.pi
		normalized_angles[JOINT2] *= -1
		if NORM_DEBUG:
			rospy.loginfo(f'**\t>> (j1 upper than 1.57) Original angle : ({j1:.2f}, {j2:.2f})')
			rospy.loginfo(f'**\t>> (j1 upper than 1.57) Normalized_angle : ({normalized_angles[JOINT1]:.2f}, {normalized_angles[JOINT2]:.2f})')
	elif normalized_angles[JOINT1] < ((-1/2) * math.pi):
		normalized_angles[JOINT1] += math.pi
		normalized_angles[JOINT2] *= -1
		if NORM_DEBUG:
			rospy.loginfo(f'**\t>> (j1 lower than -1.57) Original angle : {j1:.2f}, {j2:.2f})')
			rospy.loginfo(f'**\t>> (j1 lower than -1.57) Normalized_angle : ({normalized_angles[JOINT1]:.2f}, {normalized_angles[JOINT2]:.2f})')
	else:
		if NORM_DEBUG:
			rospy.loginfo(f'**\t>> (j1 not changed) Original angle : ({j1:.2f}, {j2:.2f})')
	
	## 서보모터 2의 회전 가능 범위인 -1/2pi에서 1/2pi를 벗어난 경우
	if normalized_angles[JOINT2] > ((1/2) * math.pi):
		normalized_angles[JOINT2] = ((1/2) * math.pi)	# 최댓값으로 지정
		if NORM_DEBUG:
			rospy.loginfo(f'**\t>> (j2 under -1.57) j2({normalized_angles[JOINT2]:.2f})')
	elif normalized_angles[JOINT2] < ((-1/2) * math.pi):
		normalized_angles[JOINT2] = ((-1/2) * math.pi)	# 최솟값으로 지정
		if NORM_DEBUG:
			rospy.loginfo(f'**\t>> (j2 over 1.57) j2({normalized_angles[JOINT2]:.2f})')
	else:
		if NORM_DEBUG:
			rospy.loginfo(f'**\t>> (j2 in range) j2({normalized_angles[JOINT2]:.2f})')

	return normalized_angles


def make_test_point_marker(coord: list) -> Marker:
	"""
	테스트 모드에서 테스트 좌표를 마커(점)으로 반환하는 함수
	"""
	marker = Marker()
	marker.header.frame_id = "base_link"
	marker.header.stamp = rospy.Time.now()
	marker.ns = "test_coord"
	marker.id = 0
	marker.type = Marker.POINTS
	marker.action = Marker.ADD
	
	marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
	marker.scale.x = 0.1
	marker.scale.y = 0.1

	point = Point()
	point.x = coord[X]
	point.y = coord[Y]
	point.z = coord[Z]
	marker.points.append(point)

	marker.header.stamp = rospy.Time.now()
	return marker


def target_callback(msg: Object) -> None:
	"""
	타겟 객체의 위치 정보를 토대로 각 서보모터의 회전 각도를 계산하고 발행하는 함수
	"""
	## 테스트용 코드
	global TEST_COORD

	coord_for_servo1 = rotate_coord_for_servo1(msg.position)		# 헤드램프 전체 좌표계에 맞게 회전한 좌표
	coord_for_servo2 = rotate_coord_for_servo2(coord_for_servo1)	# 2번 서보모터의 좌표계에 맞게 회전한 좌표(2번 서보모터 각도 계산에만 사용)

	if TEST:
		marker_pub = rospy.Publisher('/headlamp/test_coord', Marker, queue_size=10)
		TEST_COORD[X] += 0.001
		TEST_COORD[Y] += 0.025
		#TEST_COORD[Z] -= 0.3
		#TEST_COORD = [3, -2, 5]
		marker = make_test_point_marker(TEST_COORD)
		coord_for_servo1 = rotate_coord_for_servo1(TEST_COORD)
		coord_for_servo2 = rotate_coord_for_servo2(coord_for_servo1)
		
		rospy.loginfo(f'**\t>> Object position(to car) : ({TEST_COORD[X]:.2f}, {TEST_COORD[Y]:.2f}, {TEST_COORD[Z]:.2f})')
	elif DEBUG:
		rospy.loginfo(f'**\t>> Object position(to car) : ({msg.position[X]:.2f}, {msg.position[Y]:.2f}, {msg.position[Z]:.2f})')

	if DEBUG:
		rospy.loginfo(f'**\t>> Object position(to arm) : ({coord_for_servo1[X]:.2f}, {coord_for_servo1[Y]:.2f}, {coord_for_servo1[Z]:.2f})')

	if DEBUG:
		rospy.loginfo(f'**\t>> Object position(to servo1) : ({coord_for_servo1[X]:.2f}, {coord_for_servo1[Y]:.2f}, {coord_for_servo1[Z]:.2f})')
		rospy.loginfo(f'**\t>> Object position(to servo2) : ({coord_for_servo2[X]:.2f}, {coord_for_servo2[Y]:.2f}, {coord_for_servo2[Z]:.2f})')

	angle = JointState()
	angle_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
	rate = rospy.Rate(10)
	
	angle.header.frame_id = ''
	angle.header.stamp = rospy.Time.now()
	angle.velocity = []
	angle.effort = []

	# math.atan2(y, x) 함수가 반환하는 값의 범위는 -pi와 pi 사이
	joint1 = math.atan2(-coord_for_servo1[X], coord_for_servo1[Y])
	joint2 = -math.atan2(-coord_for_servo2[Y], -coord_for_servo2[X])
	if coord_for_servo1[Y] > 0:	# 목표가 (차를 기준으로) 바닥보다 밑에 있는 경우
		joint2 *= -1
	angle.position = normalize_radian_angle_for_servo(joint1, joint2)	# JointState의 position은 라디안 단위
	angle.name = ['joint1', 'joint2']

	if DEBUG:
		rospy.loginfo(f'**\t>> Joint angle(RA) : {angle.position[JOINT1]:.2f}, {angle.position[JOINT2]:.2f}')
	
	if TEST:
		marker_pub.publish(marker)

	angle_pub.publish(angle)
	rate.sleep()
	
	
if __name__ == '__main__':
	rospy.init_node('calculate_angle')
	subscriber = rospy.Subscriber('/headlamp/target_object', Object, target_callback) 
	rospy.spin()