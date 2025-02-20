#! /usr/bin/env python3

import math
import rospy
from zed_interfaces.msg import ObjectsStamped, Object
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


DEBUG = False	## 디버그 모드
EGO_POSITION = (0, 0, 0)
g_tgt_id = -1	## id는 Object의 필드 instance_id를 의미
X = 0
Y = 1
Z = 2


def get_obj_by_id(objs: list, id: int) -> Object:
	"""Object 리스트에서 특정 instance_id에 해당하는 객체를 반환하는 함수
	
	Parameters
	----------
	objs
		탐색할 객체의 전체 목록(Object[] 리스트)
	id
		탐색할 id

	"""
	for obj in objs:
		if obj.instance_id == id:
			return obj
	return None


def get_nearest_obj(objs: list) -> Object:
	"""Object 리스트에서 기준 좌표(EGO_POSITION)와 가장 가까운 객체를 반환하는 함수

	Parameters
	----------
	objs
		거리를 계산할 객체의 전체 목록(Object[] 리스트)

	"""
	nearest_obj = None
	min_distance = float('inf')
	person_objs = [obj for obj in objs if obj.label == "Person"]
	
	for obj in person_objs:
		obj_position = (obj.position[0], obj.position[1], obj.position[2])
		distance = math.sqrt(sum((ego - obj) ** 2 for ego, obj in zip(EGO_POSITION, obj_position)))

		if distance < min_distance:
			min_distance = distance
			nearest_obj = obj
	if DEBUG:
		if nearest_obj:
			rospy.loginfo(f'*\t>> Target Object id : {nearest_obj.instance_id}, Distance : {min_distance}')

	return nearest_obj


def draw_line(p: Point) -> Marker:
	"""
	3차원 공간 상의 한 점과 원점 (0,0,0)을 잇는 가이드선을 시각화하기 위해 직선 마커를 만드는 함수
	"""
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
	line.points.append(start_point)

	end_point = Point()
	end_point.x = p.x
	end_point.y = p.y
	end_point.z = p.z
	line.points.append(end_point)

	return line


def objects_callback(msg: ObjectsStamped) -> None:
	"""
	카메라가 인식한 객체 정보를 받아 차와의 거리 또는 직전의 타겟을 근거로 타겟을 설정하고 타겟 객체의 정보 및 가이드선을 발행하는 함수
	"""
	global g_tgt_id
	objs_list = msg.objects
	id_list = [obj.instance_id for obj in objs_list]
	target = None
	tgt_pub = rospy.Publisher('/headlamp/target_object', Object, queue_size=1)
	rate = rospy.Rate(10)
	
	if g_tgt_id in id_list:	# 타겟이 시야 범위 안에 계속 존재하는 경우
		target = get_obj_by_id(objs_list, g_tgt_id)
	else:
		## 타겟이 정해지지 않았거나 타겟이 시야에서 사라진 경우
		target = get_nearest_obj(objs_list)
		g_tgt_id = target.instance_id

	guide_pub = rospy.Publisher('/headlamp/target_object/guide_line', Marker, queue_size=1)
	pnt = Point()
	pnt.x = target.position[X]
	pnt.y = target.position[Y]
	pnt.z = target.position[Z]

	guide_pub.publish(draw_line(pnt))
	tgt_pub.publish(target)
	rate.sleep()
	
	
if __name__ == '__main__':
	rospy.init_node('select_target')
	subscriber = rospy.Subscriber('/zed2i/zed_node/obj_det/objects', ObjectsStamped, objects_callback) 
	rospy.spin()
