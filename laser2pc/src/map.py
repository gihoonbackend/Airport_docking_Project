#!/usr/bin/env python3
# coding: utf-8
import rospy
import tf
from tf.transformations import quaternion_from_euler

def broadcast_map_to_odom():
    rospy.init_node('map_to_odom_broadcaster', anonymous=True)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        translation = (0.0, 0.0, 0.0)
        rotation = quaternion_from_euler(0, 0, 0)
        
        br.sendTransform(translation,
                         rotation,
                         current_time,
                         "odom",
                         "map")
        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_map_to_odom()
    except rospy.ROSInterruptException:
        pass
