#!/usr/bin/env python3
# coding: utf-8
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist
import tf2_ros
import tf2_geometry_msgs
import numpy as np

class RobotMover:
    def __init__(self):
        rospy.init_node('robot_mover', anonymous=True)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        rospy.Subscriber('/Bottom_point', Marker, self.bottom_point_callback)
        rospy.Subscriber('/Top_point', Marker, self.top_point_callback)
        
        self.bottom_point = None
        self.top_point = None

        self.bottom_point_reached = False

        # 속도 조절 변수
        self.linear_speed = 0.05  # m/s
        self.angular_speed = 0.1  # rad/s

        self.initial_bottom_point_received = False
        self.initial_top_point_received = False

    def move_to_target(self, target_point, stop_at_target=True):
        vel_msg = Twist()

        # Calculate distance and angle to the target point
        distance = np.sqrt(target_point.x**2 + target_point.y**2)
        angle_to_target = np.arctan2(target_point.y, target_point.x)

        rospy.loginfo(f"Moving to target: x={target_point.x}, y={target_point.y}, z={target_point.z}, distance={distance}, angle={angle_to_target}")

        # Rotate towards the target point
        vel_msg.angular.z = self.angular_speed if angle_to_target > 0 else -self.angular_speed
        self.velocity_publisher.publish(vel_msg)
        rospy.sleep(abs(angle_to_target) / self.angular_speed)

        # Move straight to the target point
        vel_msg.linear.x = self.linear_speed
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.sleep(distance / self.linear_speed)

        if stop_at_target:
            # Stop the robot
            vel_msg.linear.x = 0
            self.velocity_publisher.publish(vel_msg)

    def bottom_point_callback(self, msg):
        if not self.initial_bottom_point_received:
            self.bottom_point = msg.pose.position
            self.initial_bottom_point_received = True
            self.transform_and_move(self.bottom_point, "bottom")

    def top_point_callback(self, msg):
        if not self.initial_top_point_received:
            self.top_point = msg.pose.position
            self.initial_top_point_received = True

    def transform_and_move(self, point, point_type):
        point_stamped = tf2_geometry_msgs.PointStamped()
        point_stamped.header.frame_id = "base_scan"  # Replace with the frame ID of your input cloud
        point_stamped.point = point

        try:
            transform = self.tf_buffer.lookup_transform("map", "base_scan", rospy.Time(0), rospy.Duration(1.0))  # Replace "map" with your target frame
            transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            rospy.loginfo(f"Transformed {point_type} point coordinates: x={transformed_point.point.x}, y={transformed_point.point.y}, z={transformed_point.point.z}")
            
            if point_type == "bottom":
                self.move_to_target(transformed_point.point)
                self.bottom_point_reached = True
                rospy.sleep(3)  # Wait for 3 seconds before moving to the top point
                if self.top_point:
                    self.align_and_move_to_top_point()

        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup failed: {e}")
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr(f"Transform extrapolation failed: {e}")

    def align_and_move_to_top_point(self):
        top_point_stamped = tf2_geometry_msgs.PointStamped()
        top_point_stamped.header.frame_id = "base_scan"  # Replace with the frame ID of your input cloud
        top_point_stamped.point = self.top_point

        try:
            transform = self.tf_buffer.lookup_transform("map", "base_scan", rospy.Time(0), rospy.Duration(1.0))  # Replace "map" with your target frame
            transformed_top_point = tf2_geometry_msgs.do_transform_point(top_point_stamped, transform)
            rospy.loginfo(f"Transformed top point coordinates: x={transformed_top_point.point.x}, y={transformed_top_point.point.y}, z={transformed_top_point.point.z}")

            # Align to the top point
            angle_to_top_point = np.arctan2(transformed_top_point.point.y, transformed_top_point.point.x)
            vel_msg = Twist()

            # Rotate towards the top point
            vel_msg.angular.z = self.angular_speed if angle_to_top_point > 0 else -self.angular_speed
            self.velocity_publisher.publish(vel_msg)
            rospy.sleep(abs(angle_to_top_point) / self.angular_speed)

            # Move straight to the top point
            distance_to_top_point = np.sqrt(transformed_top_point.point.x**2 + transformed_top_point.point.y**2)
            vel_msg.linear.x = self.linear_speed
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            rospy.sleep(distance_to_top_point / self.linear_speed)

            # Stop the robot
            vel_msg.linear.x = 0
            self.velocity_publisher.publish(vel_msg)

        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup failed: {e}")
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr(f"Transform extrapolation failed: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    mover = RobotMover()
    mover.run()
