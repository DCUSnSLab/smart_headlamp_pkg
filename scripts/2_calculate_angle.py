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

NORM_DEBUG = True
DEBUG = True	## 디버그 모드
TEST = True		## 테스트 모드
TEST_COORD = [0, 0, 0]
JOINT1 = 0
JOINT2 = 1
X = 0
Y = 1
Z = 2
HALF_PI = (1.00 / 2.00) * math.pi
PI = math.pi


def amplify_coord(coord: list) -> list:
	"""
	소수점 2번째 자리까지는 유의미한 변화를 보이지 않는 좌표의 원활한 기울기 계산을 위해 값을 100배 키워주는 함수
	높이가 0보다 낮은 경우(객체가 땅바닥을 뚫은 경우) 증폭 후 높이를 0.5로 초기화
	"""
	coord_100x = []
	
	for c in coord:
		coord_100x.append(c * 100.00)


	if coord[Z] <= 0.0:
		coord_100x[Z] = 0.5

	return coord_100x


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
	#orig = np.array(amplify_coord(coord))
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


def normalize_radian_angle_for_servo(j1: float, j2: float, coord: list) -> list:
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
	g_coord = np.array(coord).tolist()

	## 서보모터 2의 회전 가능 범위인 (-half_pi ~ -pi) + pi와 (half_pi ~ pi) - pi를 벗어난 경우
	if -HALF_PI < normalized_angles[JOINT2] < 0:
		if NORM_DEBUG:
			rospy.loginfo(f'**\t>> (-1.57 < j2 < 0) j2({normalized_angles[JOINT2]:.2f}) => -1.57')
		normalized_angles[JOINT2] = -HALF_PI
	elif 0 < normalized_angles[JOINT2] < HALF_PI:
		if NORM_DEBUG:
			rospy.loginfo(f'**\t>> (0 < j2 < 1.57) j2({normalized_angles[JOINT2]:.2f}) => 1.57')
		normalized_angles[JOINT2] = HALF_PI

	## 서보모터 2의 회전각 일반화(-half_pi ~ half_pi)
	if -PI <= normalized_angles[JOINT2] < -HALF_PI:
		normalized_angles[JOINT2] += PI
		if NORM_DEBUG:
			rospy.loginfo(f'**\t>> (-3.14 <= j2 < -1.57) -> j2({normalized_angles[JOINT2]-PI:.2f}) + PI => {normalized_angles[JOINT2]:.2f}')
	elif HALF_PI < normalized_angles[JOINT2] <= PI:
		normalized_angles[JOINT2] -= PI
		if NORM_DEBUG:
			rospy.loginfo(f'**\t>> (1.57 < j2 <= 3.14) -> j2({normalized_angles[JOINT2]+PI:.2f}) - PI => {normalized_angles[JOINT2]:.2f}')

	## Z값이 양수일 때 180도 뒤집힌 방향을 가리키는 버그 발생
	if g_coord[Z] > 0:
		normalized_angles[JOINT2] *= -1

	## 서보모터 1의 회전 가능 범위인 (-half_pi ~ half_pi)를 벗어난 경우
	# if normalized_angles[JOINT1] > HALF_PI:
	# 	normalized_angles[JOINT1] -= PI
	# 	normalized_angles[JOINT2] *= -1
	# 	if NORM_DEBUG:
	# 		rospy.loginfo(f'**\t>> (j1 > 1.57) Original angle : ({j1:.2f}, {-normalized_angles[JOINT2]:.2f}) -> ({normalized_angles[JOINT1]:.2f}, {normalized_angles[JOINT2]:.2f})')
	# elif normalized_angles[JOINT1] < -HALF_PI:
	# 	normalized_angles[JOINT1] += PI
	# 	normalized_angles[JOINT2] *= -1
	# 	if NORM_DEBUG:
	# 		rospy.loginfo(f'**\t>> (j1 < -1.57) Original angle : ({j1:.2f}, {-normalized_angles[JOINT2]:.2f}) -> ({normalized_angles[JOINT1]:.2f}, {normalized_angles[JOINT2]:.2f})')

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

	coord_for_car = msg.position

	if TEST:
		dx = rd.randint(-5, 5) * 0.03
		dy = rd.randint(-5, 5) * 0.03
		dz = rd.randint(-5, 5) * 0.01
		rospy.loginfo(f'**\t>> Random Value : dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f}')
		TEST_COORD[X] += dx
		TEST_COORD[Y] += dy
		TEST_COORD[Z] += dz
		coord_for_car = TEST_COORD
		
		rospy.loginfo(f'**\t>> Object position(to car) : ({TEST_COORD[X]:.2f}, {TEST_COORD[Y]:.2f}, {TEST_COORD[Z]:.2f})')
	elif DEBUG:
		rospy.loginfo(f'**\t>> Object position(to car) : ({msg.position[X]:.2f}, {msg.position[Y]:.2f}, {msg.position[Z]:.2f})')

	coord_for_servo1 = rotate_coord_for_servo1(coord_for_car)		# 헤드램프 전체 좌표계에 맞게 회전한 좌표
	coord_for_servo2 = rotate_coord_for_servo2(coord_for_servo1)	# 2번 서보모터의 좌표계에 맞게 회전한 좌표(2번 서보모터 각도 계산에만 사용)

	if DEBUG:
		rospy.loginfo(f'**\t>> Object position(to arm) : ({coord_for_servo1[X]:.2f}, {coord_for_servo1[Y]:.2f}, {coord_for_servo1[Z]:.2f})')

	if DEBUG:
		rospy.loginfo(f'**\t>> Object position(to servo1) : ({coord_for_servo1[X]:.2f}, {coord_for_servo1[Y]:.2f}, {coord_for_servo1[Z]:.2f})')
		rospy.loginfo(f'**\t>> Object position(to servo2) : ({coord_for_servo2[X]:.2f}, {coord_for_servo2[Y]:.2f}, {coord_for_servo2[Z]:.2f})')

	angle = JointState()
	angle_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
	marker = make_test_point_marker(coord_for_car)
	marker_pub = rospy.Publisher('/headlamp/test_coord', Marker, queue_size=10)
	rate = rospy.Rate(10)
	
	angle.header.frame_id = ''
	angle.header.stamp = rospy.Time.now()
	angle.velocity = []
	angle.effort = []

	# math.atan2(y, x) 함수가 반환하는 값의 범위는 -pi와 pi 사이
	joint1 = math.atan2(-coord_for_servo1[X], coord_for_servo1[Y])
	#joint2 = math.atan2(-coord_for_servo2[Y], -coord_for_servo2[X])	# Joint2의 XY좌표계는 (a, b) -> (-a, -b)
	joint2 = math.atan2(coord_for_servo2[Y], coord_for_servo2[X])

	angle.position = normalize_radian_angle_for_servo(joint1, joint2, coord_for_car)	# JointState의 position은 라디안 단위
	angle.name = ['joint1', 'joint2']

	if DEBUG:
		rospy.loginfo(f'**\t>> Joint1 angle(RA) : {angle.position[JOINT1]:.2f}') 
		rospy.loginfo(f'**\t>> Joint2 angle(RA) : {angle.position[JOINT2]:.2f}')
	
	#if TEST:
	marker_pub.publish(marker)

	angle_pub.publish(angle)
	rate.sleep()
	
	
if __name__ == '__main__':
	rospy.init_node('calculate_angle')
	subscriber = rospy.Subscriber('/headlamp/target_object', Object, target_callback) 
	rospy.spin()