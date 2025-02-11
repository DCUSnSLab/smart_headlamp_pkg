#! /usr/bin/env python3

import rospy
import numpy as np
from math import pi, atan, acos
from zed_interfaces.msg import ObjectsStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA

DE2RA = pi / 180.00
X = 0
Y = 1
Z = 2


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
	#return angle
	return 0.7
	
	
def translate_coord(orig: list) -> list:
	roll = 1.5708	# x
	pitch = 0.0	# y
	yaw = 1.5708	# z
	
	R_x = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
	R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
	R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
	R = R_z @ R_y @ R_x
	
	return R @ orig
	
			
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
	

def object_callback(msg):

	#print(f'> Original z : {msg.points[0].z:.4f} -> {(msg.points[0].z-0.11):.4f}')
	print(f'> Original z : {msg.objects[0].position[Z]:.4f} -> {(msg.objects[0].position[Z]-0.11):.4f}')
	rate = rospy.Rate(15)
	guide_pub = rospy.Publisher('guide_line', Marker, queue_size=1)
	
	angle_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
	angle = JointState()
	#angle.header.frame_id = ''
	angle.header.frame_id = msg.header.frame_id
	angle.header.stamp = rospy.Time.now()
	angle.name = ['joint1', 'joint2']
	
	## set tf
	tf = translate_coord(msg.objects[0].position)
	
	joint1 = calculate_xy_angle(tf[X], (tf[Y] - 0.11))
	joint2 = calculate_yz_angle((tf[Y] - 0.11), tf[Z])
	#joint2 = calculate_angle(msg.points[0].y, (msg.points[0].z - 0.11))
	#joint2 = -1 * atan(msg.points[0].y / (msg.points[0].z - 0.11))
	
	print(f'> Servo 1 : {joint1:.4f} , Servo 2 : {joint2:.4f}')
	
	angle.position = [joint1, joint2]
	angle.velocity = []
	angle.effort = []
	
	pnt = Point()
	pnt.x = tf[X]
	pnt.y = tf[Y]
	pnt.z = tf[Z]
	guide_pub.publish(draw_line(pnt))	#msg.points[0]
	angle_pub.publish(angle)
	rate.sleep()
	
	
if __name__ == '__main__':
	rospy.init_node('calculate_servo_angle')
	#subscriber = rospy.Subscriber('/tracking_point', Marker, object_callback)
	subscriber = rospy.Subscriber('/zed2i/zed_node/obj_det/objects', ObjectsStamped, object_callback) 
	rospy.spin()
