# !/usr/bin/env python
# -*-coding:utf-8-*-
"""
    Author: Yifei Ren
    Function: Differential robot controller based on polar coordinates
    Version: 1.0
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
        self.dx = self.goal_x - self.cur_x
        self.dy = self.goal_y - self.cur_y
        self.rho = 0
        self.alpha = 0
        self.beta = 0
        self.v = 0
        self.w = 0
        self.v_left = 0
        self.v_right = 0
        # Define controller parameters
        self.k_rho = 3.0
        self.k_alpha = 8.0
        self.k_beta = -1.5
        # Define differential robot parameters
        self.r = 1
        self.l = 1

        # Start creating the controller
        self.Cartesian2Polar()
        self.cal_controller()
        self.World2Vel()

        print "The velocity of the left wheel is : ", self.v_left
        print "The velocity of the right wheel is : ", self.v_right

    def Cartesian2Polar(self):
        self.rho = math.sqrt(self.dx ** 2 + self.dy ** 2)
        self.alpha = -self.cur_theta + math.atan2(self.dy, self.dx)
        self.beta = -self.cur_theta - self.alpha

    def cal_controller(self):
        self.v = self.k_rho * self.rho
        self.w = self.k_alpha * self.alpha + self.k_beta * self.beta

    def World2Vel(self):
        self.v_left = 0.5 * (2* self.v / self.r + self.w * self.l)
        self.v_right = 0.5 * (2* self.v / self.r - self.w * self.l)


if __name__ == '__main__':
    controller()