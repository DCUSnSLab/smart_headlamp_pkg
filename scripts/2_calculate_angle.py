#! /usr/bin/env python3

import numpy as np
import math
import rospy
from zed_interfaces.msg import Object
from sensor_msgs.msg import JointState


X = 0
Y = 1
Z = 2


def rotate_pitch_positive_90(coord: list) -> list:
	"""3차원 좌표를 y축을 기준으로(pitch) +90도만큼 변환(판때기가 왼쪽으로 옴)하는 함수

	Fields
	----------
	rotation_matrix
		pitch +90만큼 회전시키는 행렬
		
	Note
	----------
	이제 사용하지 않음
	"""
	rotation_matrix = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
	orig = np.array(coord)
	rotated = rotation_matrix @ orig

	return rotated.tolist()


def rotate_coord(coord: list) -> list:
	"""
	3차원 좌표를 Yaw(Z축) -90도, Roll(X축) -90도만큼 변환(판때기가 위쪽으로 옴)하는 함수
	"""
	rotation_matrix = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, -1]])
	orig = np.array(coord)
	rotated = rotation_matrix @ orig

	return rotated.tolist()


def calculate_slope_to_degree(a: float, b: float) -> float:
	"""
	원점 (0,0)과 점 (a,b)를 잇는 직선의 기울기를 라디안 단위로 계산하는 함수
	"""
	return math.atan2(b, a)


def normalize_angle_in_range(angle: float) -> float:
	"""
	입력된 각도를 -π와 π 사이로 정규화하는 함수
	"""
	while angle <= -math.pi: angle += 2 * math.pi
	while angle > math.pi: angle -= 2 * math.pi

	return angle


def normalize_joint_angle(j1: float, j2: float) -> list:
	"""
	입력된 두 각도(-π와 π 사이의 값)를 각 서보모터가 회전할 수 있는 범위의 각도로 정규화하는 함수
	"""
	if j1 < (-0.5 * math.pi):
		rospy.loginfo(f'** 2_calculate_angle\t>> Original angle : ({j1:.2f}, {j2:.2f})')
		j1 += math.pi
	elif j1 > (0.5 * math.pi):
		rospy.loginfo(f'** 2_calculate_angle\t>> Original angle : ({j1:.2f}, {j2:.2f})')
		j1 -= math.pi
	else:
		j2 *= -1
		rospy.loginfo(f'** 2_calculate_angle\t>> Original angle(not changed) : ({j1:.2f}, {j2:.2f})')
	
	if j2 > 1.0: j2 = 1.0
	elif j2 <= -1.0: j2 = -1.0

	return [j1, j2]



def target_callback(msg: Object) -> None:
	"""
	타겟 객체의 위치 정보를 토대로 각 서보모터의 회전 각도를 계산하고 발행하는 함수
	"""
	#rospy.loginfo(f'** 2_calculate_angle\t>> Object position(to car) : ({msg.position[X]:.2f}, {msg.position[Y]:.2f}, {msg.position[Z]:.2f})')
	rotated_coord = rotate_coord(msg.position)
	#rospy.loginfo(f'** 2_calculate_angle\t>> Object position(to arm) : ({rotated_coord[X]:.2f}, {rotated_coord[Y]:.2f}, {rotated_coord[Z]:.2f})')
	angle = JointState()
	angle_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
	rate = rospy.Rate(10)
	
	angle.header.frame_id = ''
	angle.header.stamp = rospy.Time.now()
	angle.velocity = []
	angle.effort = []
	
	## 디버깅용 코드
	rotated_coord = [0.00, 0.5, 0.5]
	rospy.loginfo(f'** 2_calculate_angle\t>> Object position(to arm) : ({rotated_coord[X]:.2f}, {rotated_coord[Y]:.2f}, {rotated_coord[Z]:.2f})')

	joint1 = normalize_angle_in_range(calculate_slope_to_degree(-rotated_coord[Y], rotated_coord[X]))
	joint2 = normalize_angle_in_range(calculate_slope_to_degree(-rotated_coord[Z], rotated_coord[Y]))
	angle.position = normalize_joint_angle(joint1, joint2)	# JointState의 position은 라디안 단위
	angle.name = ['joint1', 'joint2']
	rospy.loginfo(f'** 2_calculate_angle\t>> Joint angle(RA) : {angle.position[X]:.2f}, {angle.position[1]:.2f}')

	angle_pub.publish(angle)
	rate.sleep()
	
	
if __name__ == '__main__':
	rospy.init_node('calculate_angle')
	subscriber = rospy.Subscriber('/headlamp/target_object', Object, target_callback) 
	rospy.spin()