#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import numpy as np
from enum import Enum
import st_stack as stack
import st_graph as graph
import copy
'st graph class'

__auther__ = 'Xiaosheng'

ST_MAX_ACC = 100
ST_MAX_VEL = 100
MAX_ST_BOX_SIZE = 5

class St_greedy(object):
    def __init__(self):

        '''
        input_value
        '''
        self.init_vel = 0.0

        '''
        params
        '''
        self.max_time = 10
        self.step_time = 0.9
        self.acc = 1.0
        self.min_dec = 0.2
        self.step_dec = 0.2
        self.stop_delta_dec = 0.1
        self.dynamic_delta_dec = 0.1
        self.feel_safe_dec = 0.4
        self.max_dec = 2.0
        '''
        data
        '''
        self.stack = stack.St_stack()
        self.last_success_stack = stack.St_stack()
        self.graph = graph.ST_Graph()
        self.init_stack()

    def init_stack(self):
        index_t = 0
        step_time = self.step_time
        max_time = self.max_time
        while True:
            if (step_time * index_t) > max_time:
                break
            self.stack.nodes[index_t].index_t = index_t
            self.stack.nodes[index_t].t = step_time * index_t
            index_t +=1

    def is_greedy_search_statck_empty(self):
        if self.stack.top < 0:
            return True
        else:
            return False

    def push_greedy_search_stack(self):
        self.stack.top = self.stack.top + 1

    def pop_greedy_search_statck(self):
        if self.stack.top > 0:
            self.stack.top -= 1

    def greedy_search(self):
        uniform_dec = 0.0
        last_success_dec = 0.0
        uniform_dec = 0.0
        stop_time = self.max_time - self.step_time
        start_dec = self.min_dec
        last_fail_dec = self.min_dec
        dec = self.min_dec
        acc = self.acc
        while True:
            if self.is_greedy_search_statck_empty():
                dec = uniform_dec
                if dec >= self.max_dec:
                    break
                if last_success_dec != dec and dec >= self.min_dec:
                    last_fail_dec = dec
                if last_success_dec > 0.0:
                    step_dec = (last_success_dec - last_fail_dec) / 2
                    if step_dec < self.stop_delta_dec:
                        break
                    dec = last_fail_dec + step_dec
                else:
                    dec = dec + self.step_dec
                    dec = min(dec, self.max_dec)
                dec = max(dec, self.min_dec)
                uniform_dec = dec
                start_dec = dec

                if start_dec == self.min_dec:
                    end_dec = start_dec + self.dynamic_delta_dec
                    end_dec = max(end_dec, self.feel_safe_dec)
                else:
                    end_dec = start_dec
                start_search_type = stack.greedy_search_type.SEARCH_TYPE_ACC

                self.stack.nodes[self.stack.top +1].vel = self.init_vel
                self.stack.nodes[self.stack.top +1].acc = 0.0
                self.stack.nodes[self.stack.top +1].dec_to_be_used = start_dec 
                self.stack.nodes[self.stack.top +1].block_type = stack.greedy_block_type.GREEDY_NO_BLOCKED
                self.stack.nodes[self.stack.top +1].search_type = start_search_type

                for i in range(stack.greedy_search_type.SEARCH_TYPE_NUM.value -1):
                    self.stack.nodes[self.stack.top +1].search_allow[i] = True
                self.push_greedy_search_stack()

            if self.stack.nodes[self.stack.top].t > stop_time:
                if not self.is_greedy_search_statck_empty():
                    last_success_dec = start_dec
                    self.last_success_stack = copy.deepcopy(self.stack)
                    self.stack.top = -1
                    dec = last_fail_dec
                continue
            self.stack.nodes[self.stack.top +1].block_type = stack.greedy_block_type.GREEDY_NO_BLOCKED
            for i in range(stack.greedy_search_type.SEARCH_TYPE_NUM.value-1):
                self.stack.nodes[self.stack.top +1].search_allow[i] = self.stack.nodes[self.stack.top].search_allow[i]
            
            delta_time = self.stack.nodes[self.stack.top +1].t - self.stack.nodes[self.stack.top].t
            if delta_time <= 0.0:
                break

            if self.stack.nodes[self.stack.top].search_type == stack.greedy_search_type.SEARCH_TYPE_ACC:
                if self.stack.nodes[self.stack.top].search_allow[self.stack.nodes[self.stack.top].search_type.value-1]:
                    action_result_vel = self.stack.nodes[self.stack.top].vel + delta_time * acc
                    limit_vel = ST_MAX_VEL
                else:
                    limit_vel = -ST_MAX_VEL
                while True:
                    if self.stack.nodes[self.stack.top].vel >= limit_vel:
                        start_search_type = stack.greedy_search_type.SEARCH_TYPE_KEEP
                        break
                    self.stack.nodes[self.stack.top +1].vel = min(action_result_vel, limit_vel)
                    self.stack.nodes[self.stack.top +1].s = self.stack.nodes[self.stack.top].s
                    self.stack.nodes[self.stack.top +1].s += (self.stack.nodes[self.stack.top +1].vel + self.stack.nodes[self.stack.top].vel) * delta_time * 0.5
                    self.stack.nodes[self.stack.top +1].index_s = self.graph.s_to_index_s(self.stack.nodes[self.stack.top +1].s)
                    self.stack.nodes[self.stack.top +1].search_type = start_search_type
                    self.stack.nodes[self.stack.top +1].by_search_type = self.stack.nodes[self.stack.top].search_type;
                    if self.stack.nodes[self.stack.top].by_search_type == stack.greedy_search_type.SEARCH_TYPE_DEC:
                        self.stack.nodes[self.stack.top +1].search_allowed[stack.greed_search_type.SEARCH_TYPE_DEC.value -1] = False
                    self.stack.nodes[self.stack.top +1].block_type =\
                            self.stack.get_st_point_blocked_type()
                    if self.stack.nodes[self.stack.top +1].block_type.value >= stack.greedy_block_type.GREEDY_ACC_BLOCKED.value:
                        start_search_type = stack.greedy_search_type.SEARCH_TYPE_KEEP
                        break
                    self.stack.nodes[self.stack.top].acc = (self.stack.nodes[self.stack.top +1].vel - self.stack.nodes[self.stack.top].vel) / delta_time
                    self.stack.nodes[self.stack.top +1].dec_to_be_used = self.stack.nodes[self.stack.top].dec_to_be_used
                    self.push_greedy_search_stack()
                    break
                self.stack.nodes[self.stack.top -1].search_type = stack.greedy_search_type(self.stack.nodes[self.stack.top -1].search_type.value +1)
            elif self.stack.nodes[self.stack.top].search_type == stack.greedy_search_type.SEARCH_TYPE_KEEP:
                if self.stack.nodes[self.stack.top].search_allow[self.stack.nodes[self.stack.top].search_type.value -1]:
                    action_result_vel = self.stack.nodes[self.stack.top].vel
                    action_result_s = self.stack.nodes[self.stack.top].s + self.stack.nodes[self.stack.top].vel * delta_time

                    limit_vel = ST_MAX_VEL
                else:
                    limit_vel = -ST_MAX_VEL
                while True:
                    if action_result_vel >= limit_vel:
                        start_search_type = stack.greedy_search_type.SEARCH_TYPE_KEEP
                        break
                    self.stack.nodes[self.stack.top +1].vel = action_result_vel
                    self.stack.nodes[self.stack.top +1].s = action_result_s
                    self.stack.nodes[self.stack.top +1].index_s = self.graph.s_to_index_s(self.stack.nodes[self.stack.top +1].s)
                    self.stack.nodes[self.stack.top +1].search_type = start_search_type
                    self.stack.nodes[self.stack.top +1].by_search_type = self.stack.nodes[self.stack.top].search_type;
                    if self.stack.nodes[self.stack.top].by_search_type != stack.greedy_search_type.SEARCH_TYPE_KEEP:
                        self.stack.nodes[self.stack.top +1].search_allow[self.stack.nodes[self.stack.top].by_search_type.value -1] = False
                    self.stack.nodes[self.stack.top +1].block_type = self.stack.get_st_point_blocked_type()
                    if self.stack.nodes[self.stack.top +1].block_type.value >= stack.greedy_block_type.GREEDY_KEEP_BLOCKED.value:
                        start_search_type = stack.greedy_search_type.SEARCH_TYPE_KEEP
                        break
                    self.stack.nodes[self.stack.top].acc = (self.stack.nodes[self.stack.top +1].vel - self.stack.nodes[self.stack.top].vel) / delta_time
                    self.stack.nodes[self.stack.top +1].dec_to_be_used = self.stack.nodes[self.stack.top].dec_to_be_used
                    self.push_greedy_search_stack()
                    break
                self.stack.nodes[self.stack.top -1].search_type = stack.greedy_search_type(self.stack.nodes[self.stack.top -1].search_type.value +1)
            elif self.stack.nodes[self.stack.top].search_type == stack.greedy_search_type.SEARCH_TYPE_DEC:
                if self.stack.nodes[self.stack.top].search_allow[self.stack.nodes[self.stack.top].search_type.value-1]:
                    limit_vel = ST_MAX_VEL
                else:
                    limit_vel = -ST_MAX_VEL
                while True:
                    if self.stack.nodes[self.stack.top].vel >= limit_vel:
                        break
                    dec = self.stack.nodes[self.stack.top +1].dec_to_be_used 
                    action_result_vel = self.stack.nodes[self.stack.top].vel - delta_time * dec
                    action_result_vel = max(action_result_vel, 0.0)
                    curr_vel = self.stack.nodes[self.stack.top].vel
                    action_result_s = self.stack.nodes[self.stack.top].s
                    action_result_s += (curr_vel ^ 2 - action_result_vel ^ 2) / (2 * dec)
                    self.stack.nodes[self.stack.top +1].vel = action_result_vel
                    self.stack.nodes[self.stack.top +1].s = action_result_s
                    self.stack.nodes[self.stack.top +1].index_s = self.graph.s_to_index_s(action_result_s)
                    self.stack.nodes[self.stack.top +1].search_type = start_search_type
                    self.stack.nodes[self.stack.top +1].by_search_type = self.stack.nodes[self.stack.top].search_type;
                    if self.stack.nodes[self.stack.top].by_search_type == stack.greedy_search_type.SEARCH_TYPE_ACC:
                        self.stack.nodes[self.stack.top +1].search_allowed[stack.greed_search_type.SEARCH_TYPE_ACC.value -1] = False
                    self.stack.nodes[self.stack.top +1].block_type = self.stack.get_st_point_blocked_type()
                    if self.stack.nodes[self.stack.top +1].block_type.value >= stack.greedy_block_type.GREEDY_DEC_BLOCKED.value:
                        start_search_type = stack.greedy_search_type.SEARCH_TYPE_KEEP
                        break
                    self.stack.nodes[self.stack.top].acc = (self.stack.nodes[self.stack.top +1].vel - self.stack.nodes[self.stack.top].vel) / delta_time
                    self.stack.nodes[self.stack.top +1].dec_to_be_used = self.stack.nodes[self.stack.top].dec_to_be_used
                    self.push_greedy_search_stack()
                    break
                if self.stack.nodes[self.stack.top].dec_to_be_used > end_dec:
                    self.stack.nodes[self.stack.top -1].search_type = stack.greedy_search_type(self.stack.nodes[self.stack.top -1].search_type.value +1)
            else:
                self.pop_greedy_search_stack()
        if last_success_dec > 0.0:
            self.stack = self.last_success_stack


if __name__ == '__main__':
    st = St_greedy()
    st.greedy_search()
    st.stack.dump_st_stack()

