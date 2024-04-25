#!/usr/bin/env python3
# coding: utf-8

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped

class TurtlebotMover:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.bottom_subscriber = rospy.Subscriber("/bottom_point", PointStamped, self.bottom_point_callback)
        self.top_subscriber = rospy.Subscriber("/top_point", PointStamped, self.top_point_callback)

    def move_to_goal(self, x, y, frame_id="map"):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def bottom_point_callback(self, msg):
        self.move_to_goal(msg.point.x, msg.point.y)

    def top_point_callback(self, msg):
        self.move_to_goal(msg.point.x, msg.point.y)

if __name__ == "__main__":
    rospy.init_node('dynamic_coordinates_mover', anonymous=True)
    mover = TurtlebotMover()
    rospy.spin()
