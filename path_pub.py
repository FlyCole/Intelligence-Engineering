# !/usr/bin/env python
# -*-coding:utf-8-*-
"""
    Author: Yifei Ren
    Function: Publish the path calculated by A-star algorithm
    Version: 1.0
    Date: 04/05/2019
"""


import rospy
import numpy as np
import thread
import path_planning
from math import floor
import math
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped, Twist
import tf
from tf import transformations
import time
from std_msgs.msg import Header


class path_pub():
    def __init__(self):
        rospy.init_node("path_pub")
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.pos_x = 0
        self.pos_y = 0
        self.resolution = 0
        self.Map = []
        self.br = tf.TransformBroadcaster()
        self.trans = None
        self.rot = None
        self.if_start_find_path = False
        self.if_find = 0
        self.trans = None
        self.rot = None
        # self.listener = tf.TransformListener()
        self.goal_pose = PoseStamped()
        self.init_pose = PoseWithCovarianceStamped()
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("/odom", "/map", rospy.Time(0), rospy.Duration(4.0))


        self.init_pose_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.init_pose_callback)
        # self.trans, self.rot = self.listener.lookupTransform('odom', 'map', rospy.Time(0))
        self.goal_pose_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_pose_callback)


        self.path_pub = rospy.Publisher("/path", Path, queue_size=15)

        self.map_metadata_sub = rospy.Subscriber("/map_metadata", MapMetaData, self.get_map_data_callback)
        try:
            self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        except:
            print "No start point or goal point!"

        self.current_path = Path()
        self.map_metadata = MapMetaData()
        self.last_time = rospy.get_rostime()
        self.best_path = np.array([])



        while self.if_find != 3:
            self.find_path()



        rospy.sleep(1)


        # self.start_find_path()
        rospy.Rate(1)
        rospy.spin()


    # 回调函数系列
    def init_pose_callback(self, msg):
        # print "Start Point:"
        self.init_pose = msg
        self.if_find += 1
        self.br.sendTransform((self.current_odom.pose.pose.position.x, self.current_odom.pose.pose.position.y,
                               self.current_odom.pose.pose.position.z),
                              (self.current_odom.pose.pose.orientation.x, self.current_odom.pose.pose.orientation.y,
                               self.current_odom.pose.pose.orientation.z, self.current_odom.pose.pose.orientation.w),
                              rospy.Time.now(),
                              "odom",
                              "map")
        print msg

    def goal_pose_callback(self, msg):
        # print "Goal Point:"
        self.goal_pose = msg
        self.if_start_find_path = True
        self.if_find += 1
        print msg

    def get_map_data_callback(self, msg):
        self.pos_x = msg.origin.position.x
        self.pos_y = msg.origin.position.y
        self.resolution = msg.resolution

    def map_callback(self, msg):
        # print msg.header
        print "------"
        # print msg.info
        # print "------"
        # print len(msg.data)
        self.map_msg = msg
        raw = np.array(msg.data, dtype=np.int8)
        # print raw.shape
        self.Map = raw.reshape(msg.info.height, msg.info.width)

    def find_path(self):
        if self.if_start_find_path:
            print ('\033[0;32m [Information] Start find path with A* \033[0m')
            start = np.array([self.init_pose.pose.pose.position.x, self.init_pose.pose.pose.position.y])
            goal = np.array([self.goal_pose.pose.position.x, self.goal_pose.pose.position.y])
            start, goal = self.tran(start, goal)

            print "---------------------------"
            print "Start Point: ", start
            print "Goal Point", goal
            print "---------------------------"
            time.sleep(2)
            path = path_planning.AStar(self.Map, start, goal)
            path.main()
            path.path_backtrack()
            self.best_path = path.backtrack_path_array

            self.publisher()
            self.map_data_pub = self.Map.flatten()
            print len(list(self.map_data_pub))
            self.map_data_pub = list(self.map_data_pub)
            # print self.map_data_pub
            rospy.sleep(1)
            self.map_msg.data = self.map_data_pub
            self.map_sub.unregister()
            self.if_find += 1
        else:
            rospy.sleep(1)
            if self.if_find == 0:
                print ('\033[0;33m [Warning] Please set start pose\033[0m')
            elif self.if_find == 1:
                print ('\033[0;33m [Warning] Please set goal pose\033[0m')
            return

    def publisher(self):
        current_time = rospy.get_rostime()
        # print
        print "current_time", current_time
        self.current_path.header.stamp = rospy.get_rostime()
        self.current_path.header.frame_id = "map"
        #current_pose = PoseStamped()
        print "自由度", np.shape(self.best_path)
        # self.best_path[0, :] = self.best_path[0, :] * self.resolution
        # self.best_path[1, :] = (1248 - self.best_path[1, :]) * self.resolution
        # self.best_path[0, :] += self.pos_x
        # self.best_path[1, :] += self.pos_y
        curr = self.best_path[1, :] * self.resolution + self.pos_x
        self.best_path[1, :] = self.best_path[0, :] * self.resolution + self.pos_y
        self.best_path[0, :] = curr
        print self.best_path
        # rospy.sleep(2)
        for i in range(len(self.best_path[0, :])):
            if i == 0:
                continue
            else:
                current_pose = PoseStamped()

                delta_x = self.best_path[0][i] - self.best_path[0][i-1]
                delta_y = self.best_path[1][i] - self.best_path[1][i-1]
                th = math.atan2(delta_y, delta_x)
                goal_quat = tf.transformations.quaternion_from_euler(0, 0, th)
                current_pose.header.frame_id = "map"
                current_pose.pose.position.x = self.best_path[0][i]
                current_pose.pose.position.y = self.best_path[1][i]
                current_pose.pose.position.z = 0
                current_pose.pose.orientation.x = goal_quat[0]
                current_pose.pose.orientation.y = goal_quat[1]
                current_pose.pose.orientation.z = goal_quat[2]
                current_pose.pose.orientation.w = goal_quat[3]

                odom_pose = self.listener.transformPose("odom", current_pose)

                self.current_path.poses.append(odom_pose)


        print "Path planning over!!!!!!!!!!!!!!"


        # try:
        #     (self.trans, self.rot) = self.listener.lookupTransform('/odom', '/map', rospy.Time(0))
        # except:
        #     print "error"




        #print self.current_path.poses
        self.path_pub.publish(self.current_path)
        print "finish"

        #self.path_pub.publish(self.current_path)
        self.last_time = current_time
        rospy.sleep(1)

    def tran(self, start, goal):
        # start[0] = (start[0] - self.pos_x) // 0.05
        # start[1] = self.map_msg.info.height - (start[1] - self.pos_y) // 0.05
        # goal[0] = (goal[0] - self.pos_x) // 0.05
        # goal[1] = self.map_msg.info.height - (goal[1] - self.pos_y) // 0.05
        k = (start[1] - self.pos_y) // self.resolution
        start[1] = (start[0] - self.pos_x) // self.resolution
        start[0] = k
        k = (goal[1] - self.pos_y) // self.resolution
        goal[1] = (goal[0] - self.pos_x) // self.resolution
        goal[0] = k
        return start, goal
if __name__ == '__main__':
    path_pub()