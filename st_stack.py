#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import numpy as np
from enum import Enum
'st stack class'

__auther__ = 'Xiaosheng'

MAX_ST_POINT_LIST_SIZE = 20

greedy_search_type = Enum('greedy_search_type', ('SEARCH_TYPE_ACC', 'SEARCH_TYPE_KEEP', 'SEARCH_TYPE_DEC', 'SEARCH_TYPE_NUM'))

greedy_block_type = Enum('greedy_block_type', ('GREEDY_NO_BLOCKED', 'GREEDY_ACC_BLOCKED' , 'GREEDY_KEEP_BLOCKED', 'GREEDY_DEC_BLOCKED', 'GREEDY_BLOCK_NUM'))

class St_node(object):
    def __init__(self):
        self.s = 0.0
        self.t = 0.0
        self.index_s = 0
        self.index_t = 0
        self.vel = 0.0
        self.acc = 0.0
        self.dec_to_be_used = 0.0
        self.block_type = greedy_block_type.GREEDY_NO_BLOCKED
        self.search_type = greedy_search_type.SEARCH_TYPE_ACC
        self.by_search_type = greedy_search_type.SEARCH_TYPE_ACC
        self.search_allow = np.ones(shape=(greedy_block_type.GREEDY_BLOCK_NUM.value -2), dtype=bool)


class St_stack(object):
    def __init__(self):
        self.size = 0
        self.top = -1
        self.nodes = np.zeros(shape=MAX_ST_POINT_LIST_SIZE, dtype=St_node)
        
        for i in range(MAX_ST_POINT_LIST_SIZE):
            node = St_node()
            self.nodes[i] = node

    def dump_st_stack(self):
        for i in range(self.top +1):
            node = self.nodes[i]
            print("t:%.2f, s:%.2f, acc:%.2f, vel:%.2f, index_t:%d, index_s:%d"%\
                    (node.t, node.s, node.acc, node.vel, node.index_t, node.index_s))

if __name__ == '__main__':
    st = St_stack()
    st.size = 5
    st.dump_st_stack()
    block_type = greedy_block_type.GREEDY_NO_BLOCKED
    new_block_type = greedy_block_type(block_type.value+1)
    print("block_type:%s, value:%d" % (greedy_block_type.GREEDY_BLOCK_NUM, greedy_block_type.GREEDY_BLOCK_NUM.value))
    print("block_type:%s, value:%d" % (new_block_type, new_block_type.value))


