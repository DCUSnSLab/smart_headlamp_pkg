#! /usr/bin/env python3
## pubsubcp.py의 경량화 버전
## 아직은 original이므로 경량화 수정 작업 필요
## frame_id는 base_link로 변경할 것 (좌표계 동일함)

import rospy
import numpy as np
from std_msgs import *
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from zed_interfaces.msg import ObjectsStamped

class Bbox_feature:
    def __init__(self):
        # Initialize ros publisher, ros subscriber
        self.line_id = 0
        # topic where we publish
        self.box_publisher = rospy.Publisher('bounding_boxes', MarkerArray, queue_size=1)
        # subscribed Topic
        self.box_subscriber = rospy.Subscriber("/zed2/zed_node/obj_det/objects", ObjectsStamped, self.callback)
        # edit
        # self.point2_subscriber = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)

    # make 3d bounding box
    def draw_box(self, cube1, line_id):
        line_list = [Marker()] * 13
        markerArray = MarkerArray()
        box_points = np.array([cube1[0], cube1[1], cube1[2], cube1[3], cube1[0], cube1[4], cube1[5], cube1[6], cube1[7], cube1[4]])

        # define Marker().LINE_STRIP
        for l in range(len(line_list)):
            line_list[l].header.stamp = rospy.Time.now()
            line_list[l] = Marker()
            line_list[l].action = Marker().ADD
            # /zed2/zed_node/obj_det/objects => frame_id: "zed2_left_camera_frame"
            line_list[l].header.frame_id = "velodyne"
            line_list[l].ns = "line_" + str(l)
            line_list[l].pose.orientation.w = 1.0
            line_list[l].type = Marker().LINE_STRIP
            line_list[l].id = line_id
            line_list[l].scale.x = 0.05
            line_list[l].color.b = 1.0
            line_list[l].color.g = 1.0
            line_list[l].color.a = 1.0
            line_list[l].lifetime.secs = 1

        # draw 3d Bounding Box
        for i in range(1, 5):
            p = Point()
            d = Point()
            d.x = box_points[i - 1, 0]
            d.y = box_points[i - 1, 1]
            d.z = box_points[i - 1, 2]
            p.x = box_points[i, 0]
            p.y = box_points[i, 1]
            p.z = box_points[i, 2]
            line_list[i - 1].points.append(p)
            line_list[i - 1].points.append(d)

        for i in range(6, 10):
            p = Point()
            d = Point()
            d.x = box_points[i - 1, 0]
            d.y = box_points[i - 1, 1]
            d.z = box_points[i - 1, 2]
            p.x = box_points[i, 0]
            p.y = box_points[i, 1]
            p.z = box_points[i, 2]
            line_list[i - 2].points.append(d)
            line_list[i - 2].points.append(p)

        for i in range(5):
            p = Point()
            d = Point()
            d.x = box_points[i, 0]
            d.y = box_points[i, 1]
            d.z = box_points[i, 2]
            p.x = box_points[i + 5, 0]
            p.y = box_points[i + 5, 1]
            p.z = box_points[i + 5, 2]
            line_list[i + 7].points.append(d)
            line_list[i + 7].points.append(p)
        return line_list

    # Get subscribed zed_interfaces/Objectstamped
    def callback(self, data):
        max_len = len(data.objects)
        if max_len > 3: max_len = 3
        raw_point = np.empty((max_len, 8, 3))
        for i in range(max_len):    # i : object
            for j in range(0, 8):   # j : keypoints
                raw_point[i, j, 0] = data.objects[i].bounding_box_3d.corners[j].kp[0]
                raw_point[i, j, 1] = data.objects[i].bounding_box_3d.corners[j].kp[1]
                raw_point[i, j, 2] = data.objects[i].bounding_box_3d.corners[j].kp[2]
                self.line_id = data.objects[i].label_id
            print("raw_point: \n", raw_point[i])
            print("label_id: ", data.objects[i].label_id)
            self.box_publisher.publish(self.draw_box(raw_point[i], data.objects[i].label_id))
        # rospy.sleep(0.5)

def main():
    Bbox_feature()
    rospy.init_node('bounding_box_publisher')
    rospy.spin()

if __name__ == '__main__':
    while not rospy.is_shutdown():
        main()
