#! /usr/bin/env python3
## zed의 ObjectsStamped 메시지를 받아 주 객체를 선정 후 해당 객체의 중점을 Marker 형태로 Publish하는 코드 작성

import rospy
from zed_interfaces.msg import ObjectsStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

def object_callback(msg):

	rate = rospy.Rate(15)	
	pnt_pub = rospy.Publisher('/tracking_point', Marker, queue_size=1)
	
	if len(msg.objects) > 0:
		## Init Middle Point of Object as Marker
		marker = Marker()
		marker.header.frame_id = "velodyne"
		marker.header.stamp = rospy.Time.now()
		marker.ns = 'obj'
		marker.id = 1
		marker.type = Marker.POINTS
		marker.action = Marker.ADD
		
		## Set Color of Point to RED
		marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
		
		## Set Scale of Point
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		
		## Set Middle Point of Object to Marker Point
		point = Point()
		x = msg.objects[0].position[0]
		y = msg.objects[0].position[1]
		z = msg.objects[0].position[2]
		
		# X Position
		if x <= -5.00:
			point.x = -5.0
		elif x >= 5.00: 
			point.x = 5.00
		else:
			point.x = x
		
		# Y Position
		if y <= -5.00:
			point.y = -5.0
		elif y >= 5.00: 
			point.y = 5.00
		else:
			point.y = y
			
		# Z Position
		if z <= -5.00:
			point.z = -5.0
		elif z >= 5.00: 
			point.z = 5.00
		else:
			point.z = z

		marker.points.append(point)
		print(point)
		
		## Publish Tracking Point
		marker.header.stamp = rospy.Time.now()
		pnt_pub.publish(marker)
	rate.sleep()
	
if __name__ == '__main__':
	rospy.init_node('tracking_pnt_publisher')
	subscriber = rospy.Subscriber('/zed2/zed_node/obj_det/objects', ObjectsStamped, object_callback)
	rospy.spin()
