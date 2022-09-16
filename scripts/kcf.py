#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
from numpy.linalg import *
import numpy as np


if __name__ == '__main__':

    X = np.array([[0],[0]])
    P = np.array([[5,0],[0,5]])
    clp = 1
    D_x = clp
    D_y = clp
    D_xbar = clp
    D_ybar = clp
    Q = np.array([[D_x, 0], [0, D_y]])
    R = np.array([[D_xbar, 0], [0, D_ybar]])

    range_plt = 5
    fig=plt.figure(figsize=(8, 8), dpi=300)
    print("start")
    with open("/home/clp/catkin_ws/src/auto_landing/file/xy.txt") as f:
        lines = f.readlines()
    t = 0 
    origin_x = []
    origin_y = []
    list_x = []
    list_y = []
    max = 0
    sum = 0
    pre = 0
    strange = 0
    for line in lines:
        
        x,y = line.split(" ")
        x = float(x)
        y = float(y)
        origin_x.append(x)
        origin_y.append(y)
        
        # sum += abs(x - pre)
        # if (abs(x - pre) > 0.1):
        #     strange += 1
        # if (int(t) > 2):
        #     if (abs(x - pre)>max):
        #         max = abs(x - pre)
        #         print(max)
        #     pre = x
        # else:
        #     pre = x

        delt_X = np.array([[x],[y]])
        #1
        X = X + delt_X
        #2
        P = P + Q
        #3
        z = delt_X
        #4
        k = np.dot(P, inv(P+R))
        #5
        X = X + np.dot(k,(z-X))
        #6
        P = np.dot((np.identity(2)-k), P)

        list_x.append(X[0])
        list_y.append(X[1])
        t += 1
    # print("sum: ", sum/len(list_x), "strange: ", strange)
    t_list = [i for i in range(1, len(list_y)+1)]


    plt.subplot(211)
    plt.plot(t_list, origin_x ,t_list, origin_y, linewidth=1)
    #语法：plot(x轴坐标，y轴坐标，其他参数设置)
    # 设置图表标题，设置字体大小
    #函数title()给图表指定标题，参数fontsize指定了图表中文字的大小。
    #给x轴添加标签，设置字体大小
    plt.xlabel("num", fontsize=14)
    # 给y轴添加标签，设置字体大小
    plt.ylabel("value", fontsize=14)
    # 设置每个坐标轴的取值范围
    plt.axis([0, len(t_list), -range_plt, range_plt])   #[x.x,x.y,y.x,y.y]
    # tick_params()设置刻度标记的大小，设置刻度的样式
    plt.tick_params(axis='both', labelsize=14)
    # 打开matplotlib查看器，并显示绘制的图形
    
    plt.subplot(212)
    plt.plot(t_list, list_x ,t_list, list_y, linewidth=1)
    #语法：plot(x轴坐标，y轴坐标，其他参数设置)
    # 设置图表标题，设置字体大小
    #函数title()给图表指定标题，参数fontsize指定了图表中文字的大小。
    #给x轴添加标签，设置字体大小
    plt.xlabel("num", fontsize=14)
    # 给y轴添加标签，设置字体大小
    plt.ylabel("value", fontsize=14)
    # 设置每个坐标轴的取值范围
    plt.axis([0, len(t_list), -range_plt, range_plt])   #[x.x,x.y,y.x,y.y]
    # tick_params()设置刻度标记的大小，设置刻度的样式
    plt.tick_params(axis='both', labelsize=14)
    # 打开matplotlib查看器，并显示绘制的图形
    plt.show()
    print("done!")