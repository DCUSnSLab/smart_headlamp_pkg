#! /usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PointStamped

def get_marker_end_point():
    listener = tf.TransformListener()

    # 'link2'에서 'marker_link'의 상대적 위치
    relative_position = [ -3.5, 0, 0 ]  # URDF에서 정의된 상대 좌표

    while not rospy.is_shutdown():  # 계속해서 실행되도록 반복
        try:
            listener.waitForTransform('link2', 'marker_link', rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform('link2', 'marker_link', rospy.Time(0))
            marker_end_point = [
                trans[0] + relative_position[0],  # X
                trans[1] + relative_position[1],  # Y
                trans[2] + relative_position[2]   # Z
            ]
            rospy.loginfo(f'>> Marker end point: {marker_end_point}')
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo('TF Exception occurred')
        
        rospy.sleep(1)  # 1초마다 업데이트

if __name__ == '__main__':
    rospy.init_node('get_marker_end_point')
    get_marker_end_point()  # 반복 함수 실행
    rospy.spin()  # ROS 노드가 종료될 때까지 계속 실행
