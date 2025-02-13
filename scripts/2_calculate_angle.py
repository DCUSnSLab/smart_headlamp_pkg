#! /usr/bin/env python3

import numpy as np
from math import pi, atan, acos
import rospy
from zed_interfaces.msg import Object
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA


DE2RA = pi / 180.00
RA2DE = 180.00 / pi
X = 0
Y = 1
Z = 2


def rotate_pitch_negative_90(coord: list) -> list:
	rotation_matrix = np.array([[0, 0, 2], [0, 1, 0], [-1, 0, 0]])
	orig = np.array(coord)
	rotated = rotation_matrix @ original
	return rotated.tolist()


def calculate_xy_angle(coord1: float, coord2: float) -> float:
	if coord1 >= -0.05 and coord1 <= 0.05:
		return 0.0
	else:
		slope = coord2 / coord1
		if slope >= 0:
			return -1.57 + atan(slope)
		else:
			return 1.57 + atan(slope)
			

def calculate_yz_angle(y: float, z: float) -> float:
	m = float(y / (((y ** 2) + (z ** 2)) ** 0.5))
	if acos(m) > 0.0: angle = -1.57 + acos(m)
	else: angle = 1.57 + acos(m)
	
	if angle < 0.0:
		angle = -1.56
	else:
		angle = 1.56
	return angle
			

def draw_line(p: Point) -> Marker:
	line = Marker()
	line.header.frame_id = "base_link"
	line.header.stamp = rospy.Time.now()
	line.ns = 'guide_line'
	line.id = 0
	line.type = Marker.ARROW
	line.action = Marker.ADD
	line.scale.x = 0.02
	line.scale.y = 0.05
	line.scale.z = 0.1
	line.color = ColorRGBA(0.0, 1.0, 0.0, 0.5)	
	
	start_point = Point()
	start_point.x = 0.0
	start_point.y = 0.0
	start_point.z = 0.107
	
	end_point = Point()
	end_point.x = p.x
	end_point.y = p.y
	end_point.z = p.z
	
	line.points.append(start_point)
	line.points.append(end_point)
	
	return line


def object_callback(msg: Object):
	rate = rospy.Rate(15)
	guide_pub = rospy.Publisher('guide_line', Marker, queue_size=1)
	
	angle_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
	angle = JointState()
	angle.header.frame_id = ''
	angle.header.stamp = rospy.Time.now()
	angle.name = ['joint1', 'joint2']
	
	rotated_coord = rotate_pitch_negative_90(msg.position)
	
	joint1 = calculate_xy_angle(msg.position[X], msg.position[Y])
	joint2 = calculate_yz_angle(msg.objects[0].position[Y], (msg.objects[0].position[Z]) - 0.11)
	
	angle.position = [joint1, joint2]
	angle.velocity = []
	angle.effort = []
	
	pnt = Point()
	pnt.x = msg.objects[0].position[X]
	pnt.y = msg.objects[0].position[Y]
	pnt.z = msg.objects[0].position[Z]
	guide_pub.publish(draw_line(pnt))	#msg.points[0]
	angle_pub.publish(angle)
	rate.sleep()
	
	
if __name__ == '__main__':
	rospy.init_node('calculate_servo_angle')
	subscriber = rospy.Subscriber('/headlamp/target_object', Object, object_callback) 
	rospy.spin()