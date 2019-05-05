# !/usr/bin/env python
# -*-coding:utf-8-*-
"""
    Author: Yifei Ren
    Function: Differential robot controller based on non-linear fixed point
    Version: 2.0 (inherit from controller_polar)
    Date: 04/05/2019
"""


import math


class controller():
    """
    Create controller
    """
    def __init__(self):
        # Define calculation parameters
        self.cur_x = -1
        self.cur_y = -1
        self.cur_theta = 0
        self.goal_x = 0
        self.goal_y = 0
        self.goal_theta = 0
        self.dx = self.cur_x - self.goal_x
        self.dy = self.cur_x - self.goal_x
        self.dtheta = self.cur_theta - self.goal_theta
        self.v = 0
        self.w = 0
        self.v_left = 0
        self.v_right = 0
        self.e1 = 0
        self.e2 = 0
        self.e3 = 0
        # Define controller parameters
        self.k1 = 3.0
        self.k2 = 8.0
        # Define differential robot parameters
        self.r = 1
        self.l = 1

        # Start creating the controller
        self.cal_error()
        self.cal_controller()
        self.World2Vel()

        print "The velocity of the left wheel is : ", self.v_left
        print "The velocity of the right wheel is : ", self.v_right

    def cal_error(self):
        self.e1 = math.cos(self.cur_theta) * self.dx + math.sin(self.cur_theta) * self.dy
        self.e2 = -math.sin(self.cur_theta) * self.dx + math.cos(self.cur_theta) * self.dy
        self.e3 = self.dtheta

    def cal_controller(self):
        t = 0.1
        self.v = -self.k1 * self.e1
        self.w = -self.k2 * self.e3 + (self.e2 ** 2) * math.sin(t)

    def World2Vel(self):
        self.v_left = 0.5 * (2* self.v / self.r + self.w * self.l)
        self.v_right = 0.5 * (2* self.v / self.r - self.w * self.l)


if __name__ == '__main__':
    controller()