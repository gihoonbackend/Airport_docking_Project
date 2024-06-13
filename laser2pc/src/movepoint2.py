#!/usr/bin/env python3
# coding: utf-8
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist, PointStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np

class TurtlebotMover:
    def __init__(self):
        rospy.init_node('turtlebot_mover', anonymous=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.bottom_point_subscriber = rospy.Subscriber('/Bottom_point', Marker, self.bottom_point_callback)
        self.arrive_point_map_subscriber = rospy.Subscriber('/arrive_point_map', Point, self.arrive_point_map_callback)

        self.linear_speed = 0.05  # m/s
        self.angular_speed = 0.05  # rad/s
        self.distance_threshold = 0.02  # m, desired distance to stop near the target
        self.angle_threshold = 0.01  # rad, desired angle precision

        self.bottom_point = None
        self.bottom_point_received = False
        self.arrive_point = None
        self.arrive_point_received = False

    def bottom_point_callback(self, msg):
        if not self.bottom_point_received:
            self.bottom_point = msg.pose.position
            self.bottom_point_received = True
            rospy.loginfo(f"Received bottom point: x={self.bottom_point.x}, y={self.bottom_point.y}, z={self.bottom_point.z}")
            self.move_to_target(self.bottom_point, stop_at_target=True, after_move=self.wait_and_rotate_to_arrive_point)

    def arrive_point_map_callback(self, msg):
        if not self.arrive_point_received:
            self.arrive_point = msg
            self.arrive_point_received = True
            rospy.loginfo(f"Received arrive point: x={self.arrive_point.x}, y={self.arrive_point.y}, z={self.arrive_point.z}")

    def wait_and_rotate_to_arrive_point(self):
        rospy.sleep(3)  # Wait for 3 seconds
        if self.arrive_point_received:
            rospy.loginfo("Rotating to arrive point...")
            self.rotate_to_target(self.arrive_point)
            rospy.sleep(3)  # Wait for another 3 seconds after rotation
            rospy.loginfo("Moving to arrive point...")
            self.move_to_target(self.arrive_point)

    def move_to_target(self, target_point, stop_at_target=True, after_move=None):
        target_point_stamped = PointStamped()
        target_point_stamped.header.frame_id = "map"
        target_point_stamped.point = target_point

        try:
            transform = self.tf_buffer.lookup_transform("base_link", "map", rospy.Time(0), rospy.Duration(1.0))
            transformed_target_point = tf2_geometry_msgs.do_transform_point(target_point_stamped, transform)
            rospy.loginfo(f"Transformed target point: x={transformed_target_point.point.x}, y={transformed_target_point.point.y}, z={transformed_target_point.point.z}")

            vel_msg = Twist()
            distance = np.sqrt(transformed_target_point.point.x**2 + transformed_target_point.point.y**2)
            angle_to_target = np.arctan2(transformed_target_point.point.y, transformed_target_point.point.x)

            rospy.loginfo(f"Moving to target: x={transformed_target_point.point.x}, y={transformed_target_point.point.y}, z={transformed_target_point.point.z}, distance={distance}, angle={angle_to_target}")

            # Rotate towards the target point
            while abs(angle_to_target) > self.angle_threshold:
                vel_msg.angular.z = self.angular_speed if angle_to_target > 0 else -self.angular_speed
                self.velocity_publisher.publish(vel_msg)
                rospy.sleep(0.1)  # Short sleep to allow rotation
                transform = self.tf_buffer.lookup_transform("base_link", "map", rospy.Time(0), rospy.Duration(1.0))
                transformed_target_point = tf2_geometry_msgs.do_transform_point(target_point_stamped, transform)
                angle_to_target = np.arctan2(transformed_target_point.point.y, transformed_target_point.point.x)
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)

            # Move straight to the target point
            while distance > self.distance_threshold:
                vel_msg.linear.x = self.linear_speed
                self.velocity_publisher.publish(vel_msg)
                rospy.sleep(0.1)  # Short sleep to allow movement
                transform = self.tf_buffer.lookup_transform("base_link", "map", rospy.Time(0), rospy.Duration(1.0))
                transformed_target_point = tf2_geometry_msgs.do_transform_point(target_point_stamped, transform)
                distance = np.sqrt(transformed_target_point.point.x**2 + transformed_target_point.point.y**2)
                vel_msg.linear.x = 0
                self.velocity_publisher.publish(vel_msg)

            if stop_at_target:
                # Stop the robot
                vel_msg.linear.x = 0
                self.velocity_publisher.publish(vel_msg)

            if after_move is not None:
                after_move()

        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup failed: {e}")
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr(f"Transform extrapolation failed: {e}")

    def rotate_to_target(self, target_point):
        target_point_stamped = PointStamped()
        target_point_stamped.header.frame_id = "map"
        target_point_stamped.point = target_point

        try:
            transform = self.tf_buffer.lookup_transform("base_link", "map", rospy.Time(0), rospy.Duration(1.0))
            transformed_target_point = tf2_geometry_msgs.do_transform_point(target_point_stamped, transform)
            rospy.loginfo(f"Transformed target point for rotation: x={transformed_target_point.point.x}, y={transformed_target_point.point.y}, z={transformed_target_point.point.z}")

            vel_msg = Twist()
            angle_to_target = np.arctan2(transformed_target_point.point.y, transformed_target_point.point.x)

            rospy.loginfo(f"Rotating to target: angle={angle_to_target}")

            while abs(angle_to_target) > self.angle_threshold:
                vel_msg.angular.z = self.angular_speed if angle_to_target > 0 else -self.angular_speed
                self.velocity_publisher.publish(vel_msg)
                rospy.sleep(0.1)
                transform = self.tf_buffer.lookup_transform("base_link", "map", rospy.Time(0), rospy.Duration(1.0))
                transformed_target_point = tf2_geometry_msgs.do_transform_point(target_point_stamped, transform)
                angle_to_target = np.arctan2(transformed_target_point.point.y, transformed_target_point.point.x)

            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)

        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup failed: {e}")
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr(f"Transform extrapolation failed: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    mover = TurtlebotMover()
    mover.run()