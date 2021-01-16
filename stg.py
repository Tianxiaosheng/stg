#!/usr/bin/env python3
#coding:utf-8
import tkinter as tk
from tkinter import *

import matplotlib
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import st_greedy as greedy
from st_graph import stg_cost_type
import numpy as np



class Stg(tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)
        self.title("St_graph")
        self.createWdidgets()

    def createWdidgets(self):
        ''' 界面 '''
        footframe = tk.Frame(self)
        footframe['borderwidth'] = 12
        footframe['relief'] = 'raised'
        footframe.grid(column=0, row=0, sticky=(N, W, E, S))

        fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(fig, footframe)
        self.canvas.get_tk_widget().grid(column=0,row=0,sticky=(E, W))
        self.greedy = greedy.St_greedy()
        self.greedy.greedy_search()

        button1 = tk.Button(footframe, text='重画', command=self.draw_st_line)
        button1.grid(column=1, row=1, sticky=(E, W))
        button2 = tk.Button(footframe, text='退出', command=self._quit)
        button2.grid(column=2, row=2, sticky=(E, W))
        self.draw_st_line()
        self.draw_st_graph()

    def draw_st_graph(self):
        infi_point = np.zeros(shape=(1000, 2), dtype=float)
        infi_size = 0
        safe_point = np.zeros(shape=(1000, 2), dtype=float)
        safe_size = 0
        for i in range(self.greedy.graph.max_index_t +1):
            for j in range(self.greedy.graph.max_index_s +1):
                cost = self.greedy.graph.cost[i][j]
                if cost == stg_cost_type.STG_COST_INFI:
                    infi_point[infi_size][0] =\
                            self.greedy.graph.unit_s * j
                    infi_point[infi_size][1] =\
                            self.greedy.graph.unit_t * i
                    infi_size += 1
                elif cost == stg_cost_type.STG_COST_SAFE_DIST:
                    safe_point[safe_size][0] =\
                            self.greedy.graph.unit_s * j
                    safe_point[safe_size][1] =\
                            self.greedy.graph.unit_t * i
                    safe_size += 1
        x = []
        y = []
        for i in range(infi_size):
            x.append(infi_point[i][0])
            y.append(infi_point[i][1])
        self.ax.scatter(x, y, s=3, c='r')
        x = []
        y = []
        for i in range(safe_size):
            x.append(safe_point[i][0])
            y.append(safe_point[i][1])
        self.ax.scatter(x, y, s=3, c='b')
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 10)
        self.canvas.draw()

    def draw_st_line(self):
        x = []
        y = []
        for i in range(self.greedy.stack.top +1):
            node = self.greedy.stack.nodes[i]
            x.append(node.s)
            y.append(node.t)
        self.ax.clear()
        self.ax.plot(x,y)
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 10)
        self.canvas.draw()
    def _quit(self):
        ''' 退出 '''
        self.quit()
        self.destroy()

app = Stg()
app.mainloop()

