#!/usr/bin/python2.7
# -*- coding: utf-8 -*-

import rospy
import sys
import select
import termios
import tty
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import numpy as np
import sys
import rospy
# import matplotlib.pyplot as plt

X = []
Y = []
Z = []

lower = None
upper = None

def draw_rects_on_img(img, rects):
    """ draw rects on an image.
    Args:
        img: an image where the rects are drawn on.
        rects: a list of rects.
    Return:
        img_rects: an image with rects.
    """
    img_copy = img.copy()
    for rect in rects:
        x, y, w, h = rect
        cv2.rectangle(img_copy, (x,y), (x+w,y+h), (0,255,0), 2)
    return img_copy

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class Monitor:
    def __init__(self):
        rospy.init_node('mave_monitor', anonymous=True)
        rospy.Subscriber("/inspect_camera/inspect_camera",
                         Image, self.inspect_camera_callback)
        self.bridge = CvBridge()
        self.img = None
        self.img_bin = None
        self.pwd = sys.path[0]

    def inspect_camera_callback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def main_loop(self):
        global lower, upper
        i = 0
        r = rospy.Rate(1000)
        while (1):
            self.key = getKey()
            # 显示Opencv格式的图像
            if self.img.any():
                cv2.imshow("Stick", self.img)
                # lower = np.array([0, 0, 0])
                # upper = np.array([179, 200, 60])
            

                # 根据颜色空间阈值生成掩膜
                self.img_bin = cv2.inRange(self.img, lower, upper)
                # cv2.imshow("img", img)
                cv2.imshow("0Mask", self.img_bin)
                contours, _ = cv2.findContours(self.img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                rects = []
                for contour in contours:
                    x,y,w,h = cv2.boundingRect(contour)
                   
                    area = cv2.contourArea(contour)
                    if (area > 20):
                        rects.append([x,y,w,h])
                img_rects = draw_rects_on_img(self.img, rects)
                cv2.imshow("image with rects", img_rects)


            cv2.waitKey(3)

            if (self.key == 'r'):
                cv2.imwrite(self.pwd + "stick.jpg", self.img)
            if (self.key == '\x03' or i > 500):
                break
            r.sleep()


if __name__ == "__main__":
    
    # 回调函数必须要写
    def empty(i):
        global lower, upper
        # 提取滑动条的数值 共6个
        hue_min = cv2.getTrackbarPos("Hue Min", "sb")
        hue_max = cv2.getTrackbarPos("Hue Max", "sb")
        sat_min = cv2.getTrackbarPos("Sat Min", "sb")
        sat_max = cv2.getTrackbarPos("Sat Max", "sb")
        val_min = cv2.getTrackbarPos("Val Min", "sb")
        val_max = cv2.getTrackbarPos("Val Max", "sb")
        
    
        # img = cv2.imread("/home/clp/catkin_ws/src/auto_landing/scripts/tools/img.jpg")
        # imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # 颜色空间阈值
        lower = np.array([hue_min, sat_min, val_min])
        upper = np.array([hue_max, sat_max, val_max])
    



    # 读取图像
    # 参数(‘窗口标题’,默认参数)
    cv2.namedWindow("sb")
    # 设置窗口大小
    cv2.resizeWindow("sb", 640, 240)

    # 第一个参数时滑动条的名字，
    # 第二个参数是滑动条被放置的窗口的名字，
    # 第三个参数是滑动条默认值，
    # 第四个参数时滑动条的最大值，
    # 第五个参数时回调函数，每次滑动都会调用回调函数。
    cv2.createTrackbar("Hue Min", "sb", 0, 179, empty)
    cv2.createTrackbar("Hue Max", "sb", 100, 179, empty)
    cv2.createTrackbar("Sat Min", "sb", 0, 255, empty)
    cv2.createTrackbar("Sat Max", "sb", 255, 255, empty)
    cv2.createTrackbar("Val Min", "sb", 0, 255, empty)
    cv2.createTrackbar("Val Max", "sb", 60, 255, empty)
    # 调用函数
    empty(0)

    settings = termios.tcgetattr(sys.stdin)
    # input = input("Do you want to record video or capture image? (0 for video and 1 for picture)")

    monitor = Monitor()
    monitor.main_loop()

#     print(X)
#     print(Y)
#     print(Z)
#     ax = plt.axes(projection="3d")
#     # Add x, and y gridlines for the figure
#     ax.grid(b=True, color='blue', linestyle='-.', linewidth=0.5, alpha=0.3)
#     # Creating the color map for the plot
# # Creating the 3D plot
#     sctt = ax.scatter3D(X, Y, Z)

#     plt.title("3D scatter plot")
#     # display the  plot
#     plt.show()
