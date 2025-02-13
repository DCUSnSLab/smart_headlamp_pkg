#! /usr/bin/env python3

import numpy as np
import math
import rospy
from zed_interfaces.msg import Object
from sensor_msgs.msg import JointState


X = 0
Y = 1
Z = 2


def rotate_pitch_negative_90(coord: list) -> list:
	"""3차원 좌표를 y축을 기준으로(pitch) -90도만큼 변환하는 함수

	Fields
	----------
	rotation_matrix
		pitch -90만큼 회전시키는 행렬

	"""
	rotation_matrix = np.array([[0, 0, 2], [0, 1, 0], [-1, 0, 0]])
	orig = np.array(coord)
	rotated = rotation_matrix @ orig

	rospy.loginfo(f'** 2_calculate_angle\t>> Target position : {coord} -> {rotated.tolist()}')
	return rotated.tolist()


def calculate_slope_to_degree(a: float, b: float) -> float:
	"""
	원점 (0,0)과 점 (a,b)를 잇는 직선의 기울기를 라디안 단위로 계산하는 함수
	"""
	return math.atan2(b, a)


def target_callback(msg: Object) -> None:
	"""
	타겟 객체의 위치 정보를 토대로 각 서보모터의 회전 각도를 계산하고 발행하는 함수
	"""
	rotated_coord = rotate_pitch_negative_90(msg.position)
	angle = JointState()
	angle_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
	rate = rospy.Rate(10)
	
	angle.header.frame_id = ''
	angle.header.stamp = rospy.Time.now()
	angle.velocity = []
	angle.effort = []
	
	joint1 = calculate_slope_to_degree(rotated_coord[X], rotated_coord[Y])
	joint2 = calculate_slope_to_degree(rotated_coord[Y], rotated_coord[Z])
	angle.position = [joint1, joint2]	# JointState의 position은 라디안 단위
	angle.name = ['joint1', 'joint2']
	rospy.loginfo(f'** 2_calculate_angle\t>> Joint angle(RA) : {joint1:.2f}, {joint2:.2f}')

	angle_pub.publish(angle)
	rate.sleep()
	
	
if __name__ == '__main__':
	rospy.init_node('calculate_angle')
	subscriber = rospy.Subscriber('/headlamp/target_object', Object, target_callback) 
	rospy.spin()