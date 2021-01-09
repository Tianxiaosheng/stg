#!/usr/bin/env python3
#coding:utf-8
import tkinter as tk
from tkinter import *

import matplotlib
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk
from matplotlib.figure import Figure

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

        button1 = tk.Button(footframe, text='重画', command=self.draw)
        button1.grid(column=1, row=1, sticky=(E, W))
        button2 = tk.Button(footframe, text='退出', command=self._quit)
        button2.grid(column=2, row=2, sticky=(E, W))
        self.draw()

    def draw(self):
        ''' 绘图逻辑 '''
        x = np.random.randint(0, 50, size = 100)
        y = np.random.randint(0, 50, size = 100)
        #self.fig.clf()                       #方式1: (1)清除整个figure区域
        #self.ax = self.fig.add_subplot(111)         #(2)重新分配axes区域      
        self.ax.clear()                       #方式2:    清除原来的axes区域
        self.ax.scatter(x, y, s=3)                   #   重新画
        self.canvas.draw()
    def _quit(self):
        ''' 退出 '''
        self.quit()
        self.destroy()

app = Stg()

app.mainloop()

