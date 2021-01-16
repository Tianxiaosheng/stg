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
        self.safe_dist = 10.0
        self.upper_t = 10.0
        self.lower_t = 0.0
        self.lower_s = 30.0
        self.upper_s = 34.0

if __name__ == '__main__':
    st = St_box()
    print('obj_length:%f' % st.safe_dist)

