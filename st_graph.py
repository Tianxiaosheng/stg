#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import numpy as np
from enum import Enum
import st_box as box
from st_stack import greedy_block_type

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
        index_s = round(s / self.unit_s)
        if index_s >= self.max_index_s:
            return self.max_index_s
        else:
            return index_s

    def t_to_index_t(self, t):
        index_t = round(t / self.unit_t)
        if index_t >= self.max_index_t:
            return self.max_index_t
        else:
            return index_t

    def index_s_to_s(self, index_s):
        return index_s * self.unit_s

    def index_t_to_t(self, index_t):
        return index_t * self.unit_t
    def smooth_sigmoid(self, x):
        return (1.0 / (1.0 + np.exp(-x)))

    def is_vel_escape_succeed(self, ego_vel, dist_to_obj, dist_to_follow_line, obj_vel,\
            vel_acc_escape_time, vel_dec_escape_time, max_depth_penalty_dec, curr_acc,\
            half_penalty_depth_perc, shrink_width_of_depth_penalty):
        if (ego_vel - obj_vel) >= 0.0 and curr_acc >= 0.0:
            return False
        follow_dist = dist_to_follow_line + dist_to_obj
        intrusive_dist = 0.0
        if curr_acc < 0.0:
            vel_escape_time = vel_dec_escape_time
            max_acc_time = -ego_vel / curr_acc
            escape_dist = ego_vel * max_acc_time + 0.5 * curr_acc * pow(max_acc_time, 2)
            escape_dist = obj_vel * max_acc_time - escape_dist
            if ego_vel > obj_vel:
                vel_equal_t = (obj_vel - ego_vel) / curr_acc
                intrusive_dist = 0.5 * (ego_vel - obj_vel) * vel_equal_t
                if intrusive_dist >= dist_to_obj:
                    return False
            if dist_to_follow_line > escape_dist:
                space_escape_time = (dist_to_follow_line - escape_dist) / (obj_vel + 0.000001)
                space_escape_time += max_acc_time
            else:
                tmp_sqrt = pow(obj_vel - ego_vel, 2) - 2 * curr_acc * dist_to_follow_line
                if tmp_sqrt < 0.0:
                    return False
                space_escape_time = (obj_vel - ego_vel - pow(tmp_sqrt, 0.5))/curr_acc

                if space_escape_time < 0.0 or space_escape_time > max_acc_time:
                    return False
        else:
            vel_escape_time = vel_acc_escape_time
            if curr_acc == 0.0:
                max_acc_time = 100000.0
            else:
                max_acc_time = (obj_vel - ego_vel) / curr_acc
            max_acc_time = min(max_acc_time, vel_escape_time)
            escape_dist =\
                    ego_vel * max_acc_time + 0.5 * curr_acc * pow(max_acc_time, 2)
            escape_dist = obj_vel * max_acc_time - escape_dist
            if dist_to_follow_line > escape_dist:
                delta_vel = obj_vel - (ego_vel + curr_acc * max_acc_time)
                space_escape_time = (dist_to_follow_line - escape_dist) / delta_vel
                space_escape_time += max_acc_time
            else:
                space_escape_time = max_acc_time
        depth_penalty = (dist_to_follow_line + intrusive_dist) / follow_dist
        depth_penalty = (depth_penalty - half_penalty_depth_perc) *\
                shrink_width_of_depth_penalty
        depth_penalty = self.smooth_sigmoid(depth_penalty)
        depth_penalty *= max_depth_penalty_dec
        if depth_penalty < 0.0:
            depth_penalty = max_depth_penalty_dec
        depth_penalty = min(depth_penalty, max_depth_penalty_dec)
        if (ego_vel + vel_escape_time * curr_acc) >\
                (obj_vel - depth_penalty * space_escape_time):
            return False
        return True

    def get_block_type_of_vel_escape(self, ego_vel, obj_vel, ego_acc,\
            dist_to_follow_line, dist_to_obj, vel_acc_escape_time, vel_dec_escape_time,\
            depth_penalty_dec, half_penalty_depth_perc, shrink_width_of_depth_penalty):
        vel_escape_succeed = self.is_vel_escape_succeed(ego_vel, dist_to_obj, dist_to_follow_line,\
                obj_vel, vel_acc_escape_time, vel_dec_escape_time, depth_penalty_dec, ego_acc,\
                half_penalty_depth_perc, shrink_width_of_depth_penalty)
        if vel_escape_succeed:
            return greedy_block_type.GREEDY_NO_BLOCKED
        else:
            return greedy_block_type.GREEDY_DEC_BLOCKED

    def get_st_point_blocked_type(self, curr_node, next_node, vel_acc_escape_time,\
            vel_dec_escape_time, depth_penalty_dec, half_penalty_depth_perc,\
            shrink_width_of_depth_penalty):
        next_node.block_type = greedy_block_type.GREEDY_NO_BLOCKED
        node_block_type = greedy_block_type.GREEDY_NO_BLOCKED
        delta_s = next_node.s - curr_node.s
        delta_t = next_node.t - curr_node.t
        if delta_t == 0.0:
            print("Error:next_node->t and curr_node->t cannot be equal!")
            return greedy_block_type.GREEDY_DEC_BLOCKED

        calc_acc = (next_node.vel - curr_node.vel) / delta_t
        v = delta_s / delta_t
        t = curr_node.t
        s = curr_node.s
        last_t = t
        last_s = s
        index_t = curr_node.index_t
        index_s = curr_node.index_s
        print("curr_t:%f, curr_s:%f, next_t:%f, next_s:%f" %
                (curr_node.t, curr_node.s, next_node.t, next_node.s))
        t += 0.5 * self.unit_t
        while True:
            if t > next_node.t:
                t = next_node.t
            if t >= self.total_t:
                break
            if last_s >= self.total_s:
                break
            delta_t = t - last_t
            s += v * delta_t
            last_t = t
            for last_s in np.arange(last_s, s, self.unit_s):
                if self.total_s <= last_s:
                    break
                stg_cost = self.cost[index_t][index_s]
                stg_vel = self.vel[index_t][index_s]
                dist_to_obj = self.dist_to_obj[index_t][index_s]
                dist_to_follow_line = self.dist_to_follow_line[index_t][index_s]
                if stg_cost == stg_cost_type.STG_COST_INFI:
                    block_type = greedy_block_type.GREEDY_DEC_BLOCKED
                elif stg_cost == stg_cost_type.STG_COST_SAFE_DIST:
                    block_type = self.get_block_type_of_vel_escape(v, stg_vel,\
                            calc_acc, dist_to_follow_line, dist_to_obj,\
                            vel_acc_escape_time, vel_dec_escape_time,\
                            depth_penalty_dec, half_penalty_depth_perc,\
                            shrink_width_of_depth_penalty)
                else :
                    block_type = greedy_block_type.GREEDY_NO_BLOCKED

                node_block_type = greedy_block_type(max(node_block_type.value, block_type.value))
                if node_block_type.value >= greedy_block_type.GREEDY_DEC_BLOCKED.value:
                    return node_block_type
                index_s += 1
            if delta_t > 0.0:
                index_t += 1
            if t >= next_node.t:
                break
            t += self.unit_t
        print(node_block_type)
        return node_block_type

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


