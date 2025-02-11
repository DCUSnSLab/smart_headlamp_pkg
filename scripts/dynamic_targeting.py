#! /usr/bin/env python3

import rospy
from zed_interfaces.msg import ObjectsStamped, Object
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

IDLE = 0
MOVING = 2
OFF = 0
OK = 1
SEARCHING = 2

g_interesting_obj_id = None
g_main_obj_id = None

def calculate_distance(obj: Object):
	d = 0.0
	pos = obj.position	# list [ x, y, z ]
	
	return d
	
def get_index_with_id(id_: int, objs: list):
	index = -1
	for i in range(len(objs)):
		if objs[i].instance_id == id_:
			index = i
			break
	return index

def get_id_list(objs: list):
	id_ = []
	for o in objs:
		id_.append(o.instance_id)
	return id_

def find_stopped_obj(objs: list):
	# return list [ stop?(bool), which?(id) ]
	res = [False]
	for o in objs:
		if o.action_state != MOVING:
			res[0] = True
			res.append(o.instance_id)
	return res

def objects_callback(msg: ObjectsStamped):
	global g_interesting_obj_id
	global g_main_obj_id
	rate = rospy.Rate(0.5)	# once per 2 seconds
	pub = rospy.Publisher('closest_object', Object, queue_size=1)
	obj = Object()
	
	## check number of objects
	# if whole objects is single object >>
	if len(msg.objects) <= 1:
		obj = msg.objects[0]	# publish first object
		pub.publish(obj)
		rate.sleep()
		return
	
	## check interesting object
	# if interesting object is still not moving >>
	if g_interesting_obj_id != None and msg.objects[get_index_with_id(g_interesting_obj_id, msg.objects)].action_state != MOVING:	
		obj = msg.objects[get_index_with_id(g_interesting_obj_id, msg.objects)]
		pub.publish(obj)
		rate.sleep()
		return				# track it and get objects from ZED again
	else:
		g_interesting_obj_id = None 	# stop tracking interesting object

	## check stopped object in objects
	res = find_stopped_obj(msg.objects)
	# if stopped object(s) is in whole objects >>
	if res[0]:				
		g_interesting_obj_id = res[1]							# set stopped object to interesting object
		obj = msg.objects[get_index_with_id(g_interesting_obj_id, msg.objects)]	# track it and get objects from ZED again
		pub.publish(obj)
		rate.sleep()
		return
	
	## check main object
	min_dis = 30.0	# maximum : 30.0, meter units
	# if this is first cycle or ex-main object disappeared so is not in whole objects >>
	if g_main_obj_id == None		
		for o in msg.objects:
			if calculate_distance(o) <= min_dis:
				g_main_obj_id = o.instance_id
		obj = msg.objects[get_index_with_id(g_main_obj_id, msg.objects)]
		pub.publish(obj)
		rate.sleep()
		return			
	else:
		# if main object is still in camera area >>>
		if g_main_obj_id in get_id_list(msg.objects) and msg.objects[get_index_with_id(g_main_obj_id, msg.objects)].tracking_state == OK:	
			obj = msg.objects[get_index_with_id(g_main_obj_id, msg.objects)]
			pub.publish(obj)
			rate.sleep()
			return		
		# if main object goes out of camera area but ZED still tracks it >>>
		elif g_main_obj_id in get_id_list(msg.objects) and msg.objects[get_index_with_id(g_main_obj_id, msg.objects)].tracking_state == SEARCHING:
			obj = msg.objects[get_index_with_id(g_main_obj_id, msg.objects)]
			pub.publish(obj)
			rate.sleep()
			return
		# if main object absolutely disappeared so ZED can't track it >>>
		elif g_main_obj_id not in get_id_list(msg.objects) or msg.objects[get_index_with_id(g_main_obj_id, msg.objects)].tracking_state == OFF:				 	
			g_main_obj_id = None
			rate.sleep()
			return

if __name__ == '__main__':
	rospy.init_node('target_single_object')
	subscriber = rospy.Subscriber('/zed2i/zed_node/obj_det/objects', ObjectsStamped, objects_callback) 
	rospy.spin()
	
