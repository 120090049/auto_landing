#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt

def show_zx():
    input_values = [1, 2, 3, 4, 5]
    squares = [1, 4, 9, 16, 25]
    i=[1,35,56,78,96]
    #绘制图形
    # 参数linewidth设置plot()绘制的线条的粗细
    plt.plot(input_values, squares,input_values,i, linewidth=5)
    #语法：plot(x轴坐标，y轴坐标，其他参数设置)
    # 设置图表标题，设置字体大小
    #函数title()给图表指定标题，参数fontsize指定了图表中文字的大小。
    plt.title("Square Numbers", fontsize=24)
    #给x轴添加标签，设置字体大小
    plt.xlabel("Value", fontsize=14)
    # 给y轴添加标签，设置字体大小
    plt.ylabel("Square of Value", fontsize=14)
    # 设置每个坐标轴的取值范围
    plt.axis([0, 6, 0, 100])   #[x.x,x.y,y.x,y.y]
    # tick_params()设置刻度标记的大小，设置刻度的样式
    plt.tick_params(axis='both', labelsize=14)
    # 打开matplotlib查看器，并显示绘制的图形
    plt.show()


if __name__ == '__main__':
   show_zx()