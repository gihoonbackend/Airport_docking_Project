#!/usr/bin/env python3
# coding: utf-8
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist, PointStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np

class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_subscriber = rospy.Subscriber('/line', Marker, self.marker_callback)

        self.linear_speed = 0.1  # m/s
        self.angular_speed = 0.1  # rad/s

        self.line_points = []
        self.line_received = False

    def marker_callback(self, msg):
        if not self.line_received and len(msg.points) >= 8:
            self.line_points = [msg.points[6], msg.points[7]]
            self.line_received = True
            self.follow_line()

    def follow_line(self):
        start_point_stamped = PointStamped()
        end_point_stamped = PointStamped()

        start_point_stamped.header.frame_id = "base_scan"
        start_point_stamped.point = self.line_points[0]
        end_point_stamped.header.frame_id = "base_scan"
        end_point_stamped.point = self.line_points[1]

        try:
            transform = self.tf_buffer.lookup_transform("map", "base_scan", rospy.Time(0), rospy.Duration(1.0))
            transformed_start_point = tf2_geometry_msgs.do_transform_point(start_point_stamped, transform)
            transformed_end_point = tf2_geometry_msgs.do_transform_point(end_point_stamped, transform)

            rospy.loginfo(f"Transformed start point coordinates: x={transformed_start_point.point.x}, y={transformed_start_point.point.y}, z={transformed_start_point.point.z}")
            rospy.loginfo(f"Transformed end point coordinates: x={transformed_end_point.point.x}, y={transformed_end_point.point.y}, z={transformed_end_point.point.z}")

            self.move_to_target(transformed_start_point.point)
            self.move_to_target(transformed_end_point.point)

        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup failed: {e}")
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr(f"Transform extrapolation failed: {e}")

    def move_to_target(self, target_point, stop_at_target=True):
        vel_msg = Twist()

        distance = np.sqrt(target_point.x**2 + target_point.y**2)
        angle_to_target = np.arctan2(target_point.y, target_point.x)

        rospy.loginfo(f"Moving to target: x={target_point.x}, y={target_point.y}, z={target_point.z}, distance={distance}, angle={angle_to_target}")

        vel_msg.angular.z = self.angular_speed if angle_to_target > 0 else -self.angular_speed
        self.velocity_publisher.publish(vel_msg)
        rospy.sleep(abs(angle_to_target) / self.angular_speed)

        vel_msg.linear.x = self.linear_speed
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.sleep(distance / self.linear_speed)

        if stop_at_target:
            vel_msg.linear.x = 0
            self.velocity_publisher.publish(vel_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    follower = LineFollower()
    follower.run()
