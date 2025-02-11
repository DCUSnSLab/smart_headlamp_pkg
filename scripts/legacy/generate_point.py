#! /usr/bin/env python3

import rospy
import random as rd
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

def main():
	## Init Publisher
	marker_pub = rospy.Publisher('tracking_point', Marker, queue_size=10)
	
	## Get Start Position 3D Coordinate from User
	coord = [0.0, 0.0, 0.0]
	while True:
		text = input(">> x y z : ").split()
		if len(text) == 3:
			for i in range(3): coord[i] = float(text[i])
			break
		else:
			print(f'>> You have to enter 3 numbers, but {len(text)} numbers have been entered.\n>> Please try again.')
	
	## Set Whether Move the Point
	ch = input(">> Move the Point ? (Y|N) : ")
	mv = (ch.lower() == 'y')
	print(">> Successfully generated point !")
	
	## Init Point as Marker
	marker = Marker()
	marker.header.frame_id = "base_link"
	marker.header.stamp = rospy.Time.now()
	marker.ns = "random_moving_point"
	marker.id = 0
	marker.type = Marker.POINTS
	marker.action = Marker.ADD
	
	## Set Color of Point to RED
	marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
	
	## Set Scale of Point
	marker.scale.x = 0.1
	marker.scale.y = 0.1
	
	## Set Position of Point
	point = Point()
	point.x = coord[0]
	point.y = coord[1]
	point.z = coord[2]
	marker.points.append(point)
	
	## Publish Moved Point Every Rate (10 Hz)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		if mv:
			## Move the Point Randomly
			if rd.randrange(0, 2) == 1:
				marker.points[0].x += (rd.randrange(-500, 501) * 0.0001)
			if rd.randrange(0, 2) == 1:
				marker.points[0].y += (rd.randrange(-500, 501) * 0.0001)
			if rd.randrange(0, 2) == 1: 
				marker.points[0].z += (rd.randrange(-500, 501) * 0.0001)
			if marker.points[0].z < 0: 
				marker.points[0].z = 0.0
		else:
			k = input('>> Z (Increase or Decrease) Control [I|D|q] : ').lower()
			## Move the Point with Keyboard
			if k == 'd':
				if marker.points[0].z <= 0.11: marker.points[0].z = 0.11
				else: marker.points[0].z -= 0.1
			elif k == 'i':
				marker.points[0].z += 0.1
			elif k == 'q': break
			else: pass
		## Publish Moved Point
		print(f'>> Current Position : {marker.points[0].x:.2f} ,  {marker.points[0].y:.2f} ,  {marker.points[0].z:.2f}')
		marker.header.stamp = rospy.Time.now()
		marker_pub.publish(marker)
		rate.sleep()

if __name__ == '__main__':
	## Init Node for Publishing Marker
	rospy.init_node('generated_pnt_publisher')
	main()
