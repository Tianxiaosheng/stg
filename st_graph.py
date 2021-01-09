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
        self.st_boxes = []

        self.cost = np.zeros(shape=(self.max_index_t, self.max_index_s), dtype = stg_cost_type)
        self.vel = np.ones(shape=(self.max_index_t, self.max_index_s), dtype = float) * ST_MAX_ACC
        self.dist_to_obj = np.zeros(shape=(self.max_index_t, self.max_index_s), dtype = float)
        self.dist_to_follow_line = np.zeros(shape=(self.max_index_t, self.max_index_s), dtype = float)
        for i in range(MAX_ST_BOX_SIZE):
            self.st_boxes.append(box.St_box())

    def s_to_index_s(self, s):
        return round(s / self.unit_s)

    def t_to_index_t(self, t):
        return round(t / self.unit_t)

    def index_s_to_s(self, index_s):
        return index_s * self.unit_s

    def index_t_to_t(self, index_t):
        return index_t * self.unit_t

    def dump_st_graph(self):
        for i in reversed(range(self.max_index_t)):
            for j in range(self.max_index_s):
                if (self.cost[i][j] == stg_cost_type.STG_COST_INFI):
                    print("2", end="")
                elif (self.cost[i][j] == stg_cost_type.STG_COST_SAFE_DIST):
                    print("1", end="")
                else:
                    print("0", end="")
            print("")

if __name__ == '__main__':
    st = ST_Graph()
    st.cost[2][3] = stg_cost_type.STG_COST_INFI
    st.dump_st_graph()


