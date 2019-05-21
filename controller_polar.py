# !/usr/bin/env python
# -*-coding:utf-8-*-
"""
    Author: Yifei Ren
    Function: Differential robot controller based on polar coordinates
    Version: 1.0
    Date: 04/05/2019
"""


import math
from geometry_msgs.msg import Pose
from tf import transformations

class controller():
    """
    Create controller
    """
    def __init__(self, Pose, goal):
        # Define calculation parameters
        self.pose = Pose
        self.cur_x = Pose.position.x
        self.cur_y = Pose.position.y
        self.euler2quaternion()
        print "---------------"
        print self.pose.position
        print "---------------"

        print self.euler
        self.cur_theta = self.euler[2]
        self.goal_x = goal[0]
        self.goal_y = goal[1]
        self.dx = self.goal_x - self.cur_x
        self.dy = self.goal_y - self.cur_y
        self.rho = 0
        self.alpha = 0
        self.beta = 0
        self.v = 0
        self.w = 0
        self.euler = None
        # Define controller parameters
        self.k_rho = 1.0
        self.k_alpha = 1.0
        self.k_beta = -1.5


        # Start creating the controller
        self.Cartesian2Polar()
        self.cal_controller()

        print "The v is : ", self.v
        print "The w is : ", self.w

    def Cartesian2Polar(self):
        self.rho = math.sqrt(self.dx ** 2 + self.dy ** 2)
        self.alpha = -self.cur_theta + math.atan2(self.dy, self.dx)
        self.beta = -self.cur_theta - self.alpha

    def cal_controller(self):
        if self.dx >= 0:
            self.v = self.k_rho * self.rho
            self.w = self.k_alpha * self.alpha + self.k_beta * self.beta
        else:
            self.v = -self.k_rho * self.rho
            self.w = -(self.k_alpha * self.alpha + self.k_beta * self.beta)
        return self.v, self.w

    def euler2quaternion(self):
        quaternion = [self.pose.orientation.x, self.pose.orientation.y,
                      self.pose.orientation.z, self.pose.orientation.w]
        self.euler = transformations.euler_from_quaternion(quaternion)



