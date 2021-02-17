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
        self.init_vel = 5.0

        '''
        params
        '''
        self.max_time = 10
        self.step_time = 0.9
        self.acc = 1.0
        self.min_dec = 0.2
        self.step_dec = 0.2
        self.stop_delta_dec = 0.02
        self.dynamic_delta_dec = 0.0
        self.dynamic_step_dec = 0.1
        self.feel_safe_dec = 0.4
        self.max_dec = 2.0

        self.vel_acc_escape_time = 1.0
        self.vel_dec_escape_time = 3.0
        self.depth_penalty_dec = 0.6
        self.half_penalty_depth_perc = 0.1
        self.shrink_width_of_depth_penalty = 5
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

    def pop_greedy_search_stack(self):
        if self.stack.top >= 0:
            self.stack.top -= 1
        else:
            self.stack.top = -1

    def greedy_search(self):
        last_success_dec = 0.0
        stop_time = self.max_time - self.step_time
        start_dec = 0.0
        last_fail_dec = self.min_dec
        dec = self.min_dec
        acc = self.acc
        while True:
            if self.is_greedy_search_statck_empty():
                print("-----------------------------------------------------")
                dec = start_dec
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
                start_dec = dec

                if start_dec == self.min_dec:
                    end_dec = start_dec + self.dynamic_delta_dec
                    end_dec = max(end_dec, self.feel_safe_dec)
                else:
                    end_dec = start_dec
                print("start_dec:%.2f, end_dec:%.2f" % (start_dec, end_dec))
                start_search_type = stack.greedy_search_type.SEARCH_TYPE_ACC

                self.stack.nodes[self.stack.top +1].vel = self.init_vel
                self.stack.nodes[self.stack.top +1].acc = 0.0
                self.stack.nodes[self.stack.top +1].dec_to_be_used = start_dec 
                self.stack.nodes[self.stack.top +1].block_type =\
                        stack.greedy_block_type.GREEDY_NO_BLOCKED
                self.stack.nodes[self.stack.top +1].search_type = start_search_type

                for i in range(stack.greedy_search_type.SEARCH_TYPE_NUM.value -1):
                    self.stack.nodes[self.stack.top +1].search_allow[i] = True
                self.push_greedy_search_stack()

            curr_node = self.stack.nodes[self.stack.top]
            next_node = self.stack.nodes[self.stack.top +1]

            if curr_node.t > stop_time:
                if not self.is_greedy_search_statck_empty():
                    print("last_succeed_dec:%f" % (start_dec))
                    last_success_dec = start_dec
                    self.last_success_stack = copy.deepcopy(self.stack)
                    self.stack.top = -1
                continue
            next_node.block_type = stack.greedy_block_type.GREEDY_NO_BLOCKED
            for i in range(stack.greedy_search_type.SEARCH_TYPE_NUM.value-1):
                next_node.search_allow[i] = curr_node.search_allow[i]
            
            delta_time = next_node.t - curr_node.t
            if delta_time <= 0.0:
                break
            for i in range(stack.greedy_search_type.SEARCH_TYPE_NUM.value -1):
                print("curr_top:%d, search_allow[%d]:%d" % (self.stack.top, i, curr_node.search_allow[i]))
            if curr_node.search_type == stack.greedy_search_type.SEARCH_TYPE_ACC:
                print("acc_search!, stack_top:%d, curr_t:%f" % (self.stack.top, curr_node.t))
                if curr_node.search_allow[curr_node.search_type.value-1]:
                    action_result_vel = curr_node.vel + delta_time * acc
                    limit_vel = ST_MAX_VEL
                else:
                    limit_vel = -ST_MAX_VEL
                while True:
                    if curr_node.vel >= limit_vel:
                        start_search_type = stack.greedy_search_type.SEARCH_TYPE_KEEP
                        break
                    next_node.vel = min(action_result_vel, limit_vel)
                    next_node.s = curr_node.s
                    next_node.s += (next_node.vel + curr_node.vel) * delta_time * 0.5
                    next_node.index_s = self.graph.s_to_index_s(next_node.s)
                    next_node.index_t = self.graph.t_to_index_t(next_node.t)
                    next_node.search_type = start_search_type
                    next_node.by_search_type = curr_node.search_type
                    next_node.dec_to_be_used = curr_node.dec_to_be_used
                    if curr_node.by_search_type == stack.greedy_search_type.SEARCH_TYPE_DEC:
                        next_node.search_allowed[stack.greed_search_type.SEARCH_TYPE_DEC.value -1] = False
                    next_node.block_type =\
                            self.graph.get_st_point_blocked_type(curr_node, next_node,\
                            self.vel_acc_escape_time, self.vel_dec_escape_time,\
                            self.depth_penalty_dec, self.half_penalty_depth_perc,\
                            self.shrink_width_of_depth_penalty)
                    if next_node.block_type.value >= stack.greedy_block_type.GREEDY_ACC_BLOCKED.value:
                        start_search_type = stack.greedy_search_type.SEARCH_TYPE_KEEP
                        print("acc get keep result!")
                        break
                    curr_node.acc = (next_node.vel - curr_node.vel) / delta_time
                    self.push_greedy_search_stack()
                    break
                curr_node.search_type = stack.greedy_search_type(curr_node.search_type.value +1)
            elif curr_node.search_type == stack.greedy_search_type.SEARCH_TYPE_KEEP:
                print("keep_search, curr_top:%d, curr_t:%f" % (self.stack.top, curr_node.t))
                if curr_node.search_allow[curr_node.search_type.value -1]:
                    action_result_vel = curr_node.vel
                    action_result_s = curr_node.s + curr_node.vel * delta_time

                    limit_vel = ST_MAX_VEL
                else:
                    limit_vel = -ST_MAX_VEL
                while True:
                    if action_result_vel >= limit_vel:
                        start_search_type = stack.greedy_search_type.SEARCH_TYPE_KEEP
                        break
                    next_node.vel = action_result_vel
                    next_node.s = action_result_s
                    next_node.index_s = self.graph.s_to_index_s(next_node.s)
                    next_node.index_t = self.graph.t_to_index_t(next_node.t)
                    next_node.search_type = start_search_type
                    next_node.by_search_type = curr_node.search_type
                    next_node.dec_to_be_used = curr_node.dec_to_be_used
                    if curr_node.by_search_type != stack.greedy_search_type.SEARCH_TYPE_KEEP:
                        next_node.search_allow[curr_node.by_search_type.value -1] = False
                    next_node.block_type = self.graph.get_st_point_blocked_type(curr_node, next_node,\
                            self.vel_acc_escape_time, self.vel_dec_escape_time,\
                            self.depth_penalty_dec, self.half_penalty_depth_perc,\
                            self.shrink_width_of_depth_penalty)
                    if next_node.block_type.value >= stack.greedy_block_type.GREEDY_KEEP_BLOCKED.value:
                        start_search_type = stack.greedy_search_type.SEARCH_TYPE_KEEP
                        print("keep_search get keep_bloced!")
                        break
                    curr_node.acc = (next_node.vel - curr_node.vel) / delta_time
                    self.push_greedy_search_stack()
                    break
                curr_node.search_type =\
                        stack.greedy_search_type(curr_node.search_type.value +1)
            elif curr_node.search_type == stack.greedy_search_type.SEARCH_TYPE_DEC:
                print("dec_search, top:%d, t:%f, dec:%f" % (self.stack.top, curr_node.t, curr_node.dec_to_be_used))
                if curr_node.search_allow[curr_node.search_type.value-1]:
                    limit_vel = ST_MAX_VEL
                else:
                    limit_vel = -ST_MAX_VEL
                    curr_node.dec_to_be_used = end_dec
                    print("end_dec:%f" % (end_dec))

                dec = curr_node.dec_to_be_used
                curr_node.dec_to_be_used += self.dynamic_step_dec

                while True:
                    if curr_node.vel >= limit_vel:
                        break
                    action_result_vel = curr_node.vel - delta_time * dec
                    action_result_vel = max(action_result_vel, 0.0)
                    curr_vel = curr_node.vel
                    action_result_s = curr_node.s
                    action_result_s += (curr_vel ** 2 - action_result_vel ** 2) / (2 * dec)
                    next_node.vel = action_result_vel
                    next_node.s = action_result_s
                    next_node.index_s = self.graph.s_to_index_s(action_result_s)
                    next_node.index_t = self.graph.t_to_index_t(next_node.t)
                    next_node.search_type = start_search_type
                    next_node.by_search_type = curr_node.search_type

                    if curr_node.by_search_type == stack.greedy_search_type.SEARCH_TYPE_ACC:
                        next_node.search_allow[stack.greedy_search_type.SEARCH_TYPE_ACC.value -1] = False
                    next_node.block_type = self.graph.get_st_point_blocked_type(curr_node, next_node,\
                            self.vel_acc_escape_time, self.vel_dec_escape_time, self.depth_penalty_dec,\
                            self.half_penalty_depth_perc, self.shrink_width_of_depth_penalty)
                    if next_node.block_type.value >= stack.greedy_block_type.GREEDY_DEC_BLOCKED.value:
                        start_search_type = stack.greedy_search_type.SEARCH_TYPE_KEEP
                        print("dec_search get blocked!")
                        break
                    curr_node.acc = (next_node.vel - curr_node.vel) / delta_time
                    next_node.dec_to_be_used = dec
                    self.push_greedy_search_stack()
                    break
                if curr_node.dec_to_be_used > end_dec:
                    curr_node.search_type = stack.greedy_search_type(curr_node.search_type.value +1)
            else:
                self.pop_greedy_search_stack()
        if last_success_dec > 0.0:
            self.stack = self.last_success_stack
            self.stack.dump_st_stack()


if __name__ == '__main__':
    st = St_greedy()
    st.greedy_search()
    st.stack.dump_st_stack()

