#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import numpy as np
from enum import Enum
import st_box as box

'st graph class'

__auther__ = 'Xiaosheng'

ST_MAX_ACC = 100
MAX_ST_BOX_SIZE = 5
stg_cost_type = Enum('stg_cost_type', ('STG_COST_INFI', 'STG_COST_SAFE_DIST', 'STG_COST_MIN'))

class ST_Graph(object):
    def __init__(self):
        self.total_s = 100
        self.total_t = 10
        self.unit_s = 0.6
        self.unit_t = 0.3
        self.max_index_s = int(self.total_s // self.unit_s)
        self.max_index_t = int(self.total_t // self.unit_t)
        self.st_box = box.St_box()

        self.cost = np.zeros(shape=(self.max_index_t +1,\
                self.max_index_s +1), dtype = stg_cost_type)
        self.vel = np.ones(shape=(self.max_index_t +1,\
                self.max_index_s +1), dtype = float) * ST_MAX_ACC
        self.dist_to_obj = np.zeros(shape=(self.max_index_t +1,\
                self.max_index_s +1), dtype = float)
        self.dist_to_follow_line = np.zeros(shape=(self.max_index_t +1,\
                self.max_index_s +1), dtype = float)
        self.set_graph()
        self.dump_st_graph()
        '''
        for i in range(MAX_ST_BOX_SIZE):
            self.st_boxes.append(box.St_box())
        '''

    def s_to_index_s(self, s):
        return round(s / self.unit_s)

    def t_to_index_t(self, t):
        return round(t / self.unit_t)

    def index_s_to_s(self, index_s):
        return index_s * self.unit_s

    def index_t_to_t(self, index_t):
        return index_t * self.unit_t

    def set_graph(self):
        self.fill_collision_area(self.st_box.lower_t,\
                self.st_box.upper_t, self.st_box.lower_s,\
                self.st_box.upper_s, stg_cost_type.STG_COST_INFI,\
                self.st_box.vel)

        for i in range(self.max_index_t +1):
            count = 0
            for j in range(self.max_index_s, -1, -1):
                if self.cost[i][j] == stg_cost_type.STG_COST_INFI:
                    count = self.s_to_index_s(self.st_box.safe_dist)
                    if i == 0:
                        print("count:%d"% (count))
                    self.vel[i][j] = self.st_box.vel
                    self.dist_to_obj[i][j] = 0.0
                    self.dist_to_follow_line[i][j] = self.st_box.safe_dist
                elif count > 0:
                    self.cost[i][j] = stg_cost_type.STG_COST_SAFE_DIST
                    self.vel[i][j] = self.st_box.vel
                    self.dist_to_obj[i][j] =\
                            self.st_box.safe_dist - count * self.unit_s
                    self.dist_to_follow_line[i][j] =\
                            count * self.unit_s
                    count -= 1

    def fill_collision_area(self, lower_t, upper_t, lower_s, upper_s,\
            cost_type, vel):
        start_index_t = self.t_to_index_t(lower_t)
        end_index_t = self.t_to_index_t(upper_t)

        start_s = lower_s
        end_s = upper_s
        for i in range(start_index_t, end_index_t +1):
            start_s = lower_s + vel * self.unit_t * (i - start_index_t)
            start_index_s = self.s_to_index_s(start_s)
            end_s = upper_s + vel * self.unit_t * (i - start_index_t)
            end_index_s = self.s_to_index_s(end_s)

            for j in range(start_index_s, end_index_s +1):
                self.cost[i][j] = cost_type
                if cost_type == stg_cost_type.STG_COST_INFI:
                    self.dist_to_obj[i][j] = 0.0
                    self.dist_to_follow_line[i][j] = self.total_s
                elif cost_type == stg_cost_type.STG_COST_SAFE_DIST:
                    self.dist_to_obj[i][j] = (end_index_s - j) * self.unit_s
                    self.dist_to_follow_line[i][j] =\
                            (j - start_index_s) * self.unit_s
                else:
                    self.dist_to_obj[i][j] = self.total_s
                    self.dist_to_follow_line[i][j] = 0.0

    def dump_st_graph(self):
        for i in reversed(range(self.max_index_t +1)):
            for j in range(self.max_index_s +1):
                if (self.cost[i][j] == stg_cost_type.STG_COST_INFI):
                    print("2", end="")
                elif (self.cost[i][j] == stg_cost_type.STG_COST_SAFE_DIST):
                    print("1", end="")
                else:
                    print("0", end="")
            print("")

if __name__ == '__main__':
    st = ST_Graph()
    st.set_graph()
    st.dump_st_graph()


