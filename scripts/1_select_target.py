#! /usr/bin/env python3

import rospy
from zed_interfaces.msg import ObjectsStamped, Object


X = 0
Y = 1
Z = 2


def objects_callback(msg: ObjectsStamped):
	rate = rospy.Rate(10)
	tgt_pub = rospy.Publisher('/headlamp/target_object', Object, queue_size=1)

	target = Object()

	## Filter target from object list

	tgt_pub.publish(target)
	rate.sleep()
	
	
if __name__ == '__main__':
	rospy.init_node('select_target')
	subscriber = rospy.Subscriber('/zed2i/zed_node/obj_det/objects', ObjectsStamped, objects_callback) 
	rospy.spin()
