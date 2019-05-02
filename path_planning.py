# !/usr/bin/env python
# -*-coding:utf-8-*-
"""
    Author: Ren Yifei
    Function: A* path planning
    Version: 1.0
    Date: 14/04/2019
"""

import numpy
from pylab import *
import copy


# map define
# map_grid = numpy.full((20, 20), int(10), dtype=numpy.int8)
# map_grid[3, 3:8] = 0
# map_grid[3:10, 7] = 0
# map_grid[10, 3:8] = 0
# map_grid[17, 13:17] = 0
# map_grid[10:17, 13] = 0
# map_grid[10, 13:17] = 0
# map_grid[5, 2] = 7
# map_grid[15, 15] = 5
# map_grid = numpy.full((194, 231), int(10), dtype=numpy.int8)

class AStar(object):
    """
    create A* algorithm
    """
    def __init__(self, map, start, goal):
        # self.start = numpy.array([1152 - ((36.2-1.19772148132) // 0.05), 1248 - ((31.4-2.85361003876) // 0.05)])
        #self.start = numpy.array([430, 624])
        # self.goal = numpy.array([428, 620])
        self.start = start
        self.goal = goal
        self.open = numpy.array([[], [], [], [], [], []])  # OPEN表
        self.closed = numpy.array([[], [], [], [], [], []])  # CLOSE表
        self.backtrack_path_array = numpy.array([[], []])  # 回溯路径表
        self.size = shape(map)
        self.width = self.size[1]
        self.height = self.size[0]
        self.boundary = [self.width, self.height]
        print self.boundary
        self.map_grid = map
        print self.map_grid > 0


    def h_value(self, child_node):
        """
        Calculate the expand horizons and the h value
        :param child_node:
        :return:
        """
        h = abs(child_node[0] - self.goal[0]) + abs(child_node[1] - self.goal[1])
        return h

    def g_value(self, child_node, father_node):
        """
        Calculate the g accumulation
        :param child_node:
        :param father_node:
        :return:
        """
        g = (father_node[0] - child_node[0]) ** 2 + (father_node[1] - child_node[1]) ** 2
        g = numpy.sqrt(g) + father_node[4]
        return g

    def f_value(self, child_node, father_node):
        """
        Calculate f = g + h
        :param child_node:
        :param father_node:
        :return:
        """
        f = self.g_value(child_node, father_node) + self.h_value(child_node)
        return f

    def direction(self, child_node, father_node):
        """
        Calculate the extend direction
        :param child_node:
        :param father_node:
        :return:
        """
        x = child_node[0] - father_node[0]
        y = child_node[1] - father_node[1]
        return x, y

    def location(self, search, list):
        """
        Judge whether the current searching point is in the list(OPEN/CLOSED) Then return the Judge parameter and the index
        :param search:
        :param list:
        :return:judge, index
        """
        judge = 0
        index = 0
        for i in range(list.shape[1]):
            if search[0] == list[0, i] and search[1] == list[1, i]:
                judge = judge + 1
                index = i
                break
        return judge, index

    def child_node(self, father_node):
        """
        Extend the child_node
        Update the OPEN/CLOSED list
        :param father_node:
        :return:
        """
        for i in range(-1, 2, 1):
            for j in range(-1, 2, 1):
                if i == 0 and j == 0:
                    continue
                search = [father_node[0] + i, father_node[1] + j]
                # print("The searching point is : ", search)
                #  搜索点在边界外
                if search[0] < 0 or search[0] > self.boundary[0] or search[1] < 0 or search[1] > self.boundary[1]:
                    # print "The searching point is in the boundary"
                    continue
                #  搜索点在障碍物上
                if self.map_grid[int(search[0]), int(search[1])] == 100 or self.map_grid[int(search[0]), int(search[1])] == -1 :
                    # print "The value of obstacle point is: ", self.map_grid[int(search[0]), int(search[1])]
                    continue

                current_g = self.g_value(search, father_node)  # 计算当前搜索点的g值
                current_f = self.f_value(search, father_node)  # 计算当前搜索点的f值
                delta_x, delta_y = self.direction(search, father_node)  # 记录x,y方向的变化
                parameter = [search[0], search[1], delta_x, delta_y, current_g, current_f]  # 汇总参数
                # print("Parameter : ", parameter)

                # 若在OPEN表中，则更新该点在OPEN表中参数
                j_o, index_o = self.location(search, self.open)
                if j_o == 1:
                    if current_f <= self.open[5][index_o]:
                        self.open[5][index_o] = current_f
                        self.open[4][index_o] = current_g
                        self.open[3][index_o] = delta_y
                        self.open[2][index_o] = delta_x
                    continue

                # 若在CLOSED表中，则去掉搜索点
                j_c, index_c = self.location(search, self.closed)
                if j_c == 1:
                    continue

                self.open = numpy.c_[self.open, parameter]
                # print("The current OPEN list: ")
                # print(self.open)

    def path_backtrack(self):
        """
        Backtrack the shortest path
        :return:
        """
        best_path = [self.goal[0], self.goal[1]]
        self.backtrack_path_array = numpy.array([self.goal[0], self.goal[1]])
        i = 0
        while i <= self.closed.shape[1]:
            for j in range(self.closed.shape[1]):
                if best_path[0] == self.closed[0][j] and best_path[1] == self.closed[1][j]:
                    x = self.closed[0][j] - self.closed[2][j]
                    y = self.closed[1][j] - self.closed[3][j]
                    best_path = [x, y]
                    self.backtrack_path_array = numpy.c_[self.backtrack_path_array, best_path]
                    break
                else:
                    continue
            i = i + 1
        # self.backtrack_path_array[0] = self.backtrack_path_array[0] - self.width
        # self.backtrack_path_array[1] = self.backtrack_path_array[1] - self.height
        print "The best path is as follows :"
        print self.backtrack_path_array


    def main(self):
        current = self.start
        h0 = self.h_value(current)
        init_open = [current[0], current[1], 0, 0, 0, h0]
        self.open = numpy.column_stack((self.open, init_open))

        iter = 1
        while iter:
            # If the OPEN list is empty, quit
            if self.open.shape[1] == 0:
                print("There is no path!!!!!!!!")
                return
            self.open = self.open.T[numpy.lexsort(self.open)].T  # Sort the last one row of the OPEN list
            current = self.open[:, 0]
            print('%s times current coordinate***************************' % iter)
            # print(current)
            self.closed = numpy.c_[self.closed, current]

            if current[0] == self.goal[0] and current[1] == self.goal[1]:
                print('Searching accomplished!')
                return

            self.child_node(current)
            self.open = numpy.delete(self.open, 0, axis=1)

            iter = iter + 1


class MAP(object):
    """
    Draw the map
    """
    def __init__(self, map):
        self.map_grid = map
    def draw_init_map(self):
        """
       Draw the start-goal map
        """
        plt.imshow(self.map_grid, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=10)
        xlim(-1, 2000)  # set the range of x-axis
        ylim(-1, 2000)  # set the range of y-axis
        my_x_ticks = numpy.arange(0, 20, 1)
        my_y_ticks = numpy.arange(0, 20, 1)
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        plt.grid(True)

    def draw_path_open(self, a):
        """
        Draw the point map in the OPEN list
        """
        print('Print the length of the OPEN list：')
        print(a.open.shape[1])
        map_open = copy.deepcopy(self.map_grid)
        for i in range(a.closed.shape[1]):
            x = a.closed[:, i]
            map_open[int(x[0]), int(x[1])] = 1

        plt.imshow(map_open)
        plt.imshow(map_open, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=10)
        xlim(-1, 2000)  # set the range of x-axis
        ylim(-1, 2000)  # set the range of y-axis
        my_x_ticks = numpy.arange(0, 20, 1)
        my_y_ticks = numpy.arange(0, 20, 1)
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        plt.grid(True)

    def draw_path_closed(self, a):
        """
        Draw the point map in the CLOSED list
        """
        print('Print the length of the CLOSED list：')
        print(a.closed.shape[1])
        map_closed = copy.deepcopy(self.map_grid)
        for i in range(a.closed.shape[1]):
            x = a.closed[:, i]
            map_closed[int(x[0]), int(x[1])] = 5
        plt.imshow(map_closed, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=10)
        xlim(-1, 20)  # set the range of x-axis
        ylim(-1, 20)  # set the range of y-axis
        my_x_ticks = numpy.arange(0, 20, 1)
        my_y_ticks = numpy.arange(0, 20, 1)
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        plt.grid(True)

    def draw_direction_point(self, a):
        """
        According to the recorded direction information, from the goal point, draw the searched path map
        """
        print('Print the length of direction list：')
        print(a.backtrack_path_array.shape[1])
        map_direction = copy.deepcopy(self.map_grid)
        for i in range(a.backtrack_path_array.shape[1]):
            x = a.backtrack_path_array[:, i]
            map_direction[int(x[0]), int(x[1])] = 6
        plt.imshow(map_direction, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=10)
        xlim(-1, 20)  # 设置x轴范围
        ylim(-1, 20)  # 设置y轴范围
        my_x_ticks = numpy.arange(0, 20, 1)
        my_y_ticks = numpy.arange(0, 20, 1)
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        plt.grid(True)

    def draw_three_axes(self, a):
        """
        Draw the three maps above in a figure
        """
        plt.figure()
        ax1 = plt.subplot(221)
        ax2 = plt.subplot(222)
        ax3 = plt.subplot(223)
        ax4 = plt.subplot(224)
        plt.sca(ax1)
        self.draw_init_map()
        plt.sca(ax2)
        self.draw_path_open(a)
        plt.sca(ax3)
        self.draw_path_closed(a)
        plt.sca(ax4)
        self.draw_direction_point(a)

        plt.show()


# if __name__ == '__main__':
#     path = AStar()
#     path.main()
#     path.path_backtrack()
#     m1 = MAP()
#     m1.draw_three_axes(path)
