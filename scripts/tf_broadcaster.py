#!/usr/bin/env python3
#-*-coding:utf-8 -*-
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped
import math
import rospy
import tf

if __name__ == '__main__':
    # ROS 노드를 초기화합니다.
    rospy.init_node('tf_broadcaster_node')

    # tf broadcaster를 생성합니다.
    bc = tf.TransformBroadcaster()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        bc.sendTransform((0.0, 0.0, 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0), rospy.Time.now(), "dofbot_base_link", "base_link")
        rate.sleep()
