# !/usr/bin/env python
# -*-coding:utf-8-*-
"""
    Author: Yifei Ren
    Function: Publish velocity of the mobile robot
    Version: 1.0
    Date: 07/05/2019
"""


import rospy
import std_msgs
import controller_polar
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path


class vel_pub():
    def __init__(self):
        rospy.init_node("vel_pub")
        self.vel_pub = rospy.Publisher("/vel", Twist, queue_size=5)
        self.twist = Twist()
        # self.vel_sub = rospy.Subscriber("")
        self.pose_sub = rospy.Subscriber("/water_uavcan_master/odom", Odometry, self.callback_odom)
        self.current_pose = Pose()
        self.path = Path()
        self.switch = False
        self.goal = None
        while ~self.switch:
            self.goal_sub = rospy.Subscriber("/path", Path, self.callback_path)



    def callback_odom(self, msg):
        self.current_pose = Pose()
        self.current_pose.position.x = msg.pose.pose.position.x
        self.current_pose.position.y = msg.pose.pose.position.y
        self.current_pose.position.z = msg.pose.pose.position.z
        self.current_pose.orientation.x = msg.pose.pose.orientation.x
        self.current_pose.orientation.y = msg.pose.pose.orientation.y
        self.current_pose.orientation.z = msg.pose.pose.orientation.z
        self.current_pose.orientation.w = msg.pose.pose.orientation.w

    def callback_path(self, msg):
        self.path = msg
        self.switch = True
        for i in range(len(self.path.poses)):
            self.goal = [self.path.poses[i].pose.position.x, self.path.poses[i].pose.position.y]
            while 1:
                self.control()
                if (self.current_pose.position.x - self.goal[0]) ** 2 + (self.current_pose.position.y - self.goal[1]) ** 2 < 0.0004:
                    break
                rospy.sleep(.1)

    def control(self):
        polar = controller_polar.controller(self.current_pose, self.goal)
        v, w = polar.cal_controller()
        w = w / 180 * 3.14
        self.twist.linear.x = v
        self.twist.angular.z = w
        self.vel_pub.publish(self.twist)

if __name__ == '__main__':
    vel_pub()