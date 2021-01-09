#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import numpy as np
from enum import Enum
'st box class'

__auther__ = 'Xiaosheng'

class St_box(object):
    def __init__(self):
        self.is_valid = False
        self.vel = 0.0
        self.obj_length = 4.0
        self.obj_upper_t = 0.0
        self.obj_lower_t = 0.0
        self.obj_lower_t_lower_s = 0.0
        self.obj_lower_t_upper_s = 0.0
        self.obj_upper_t_lower_s = 0.0
        self.obj_upper_t_upper_s = 0.0
    
if __name__ == '__main__':
    st = St_box()
    print('obj_length:%s' % st.obj_length)


