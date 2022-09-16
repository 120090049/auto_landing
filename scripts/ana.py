#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt




if __name__ == '__main__':
    fig=plt.figure(figsize=(4, 4), dpi=300)
    print("start")
    with open("/home/clp/catkin_ws/src/auto_landing/file/xy.txt") as f:
        lines = f.readlines()
    z = 0 
    list_x = []
    list_y = []
    for line in lines:
        z += 1
        # if (z > 60): break
        x,y = line.split(" ")
        x = float(x)
        y = float(y)
        list_x.append(x)
        list_y.append(y)
    z_list = [i for i in range(1, len(list_y)+1)]

    plt.subplot(211)
    plt.plot(z_list, list_x ,z_list, list_y, linewidth=1)
    #语法：plot(x轴坐标，y轴坐标，其他参数设置)
    # 设置图表标题，设置字体大小
    #函数title()给图表指定标题，参数fontsize指定了图表中文字的大小。
    #给x轴添加标签，设置字体大小
    plt.xlabel("num", fontsize=14)
    # 给y轴添加标签，设置字体大小
    plt.ylabel("value", fontsize=14)
    # 设置每个坐标轴的取值范围
    plt.axis([0, len(z_list), -1, 1])   #[x.x,x.y,y.x,y.y]
    # tick_params()设置刻度标记的大小，设置刻度的样式
    plt.tick_params(axis='both', labelsize=14)
    # 打开matplotlib查看器，并显示绘制的图形

    plt.subplot(212)
    plt.plot(z_list, list_x ,z_list, list_y, linewidth=1)
    #语法：plot(x轴坐标，y轴坐标，其他参数设置)
    # 设置图表标题，设置字体大小
    #函数title()给图表指定标题，参数fontsize指定了图表中文字的大小。
    #给x轴添加标签，设置字体大小
    plt.xlabel("num", fontsize=14)
    # 给y轴添加标签，设置字体大小
    plt.ylabel("value", fontsize=14)
    # 设置每个坐标轴的取值范围
    plt.axis([0, len(z_list), -1, 1])   #[x.x,x.y,y.x,y.y]
    # tick_params()设置刻度标记的大小，设置刻度的样式
    plt.tick_params(axis='both', labelsize=14)
    # 打开matplotlib查看器，并显示绘制的图形
    plt.show()
