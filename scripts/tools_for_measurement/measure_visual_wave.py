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
from std_msgs.msg import Int32

import numpy as np
import sys
import rospy
# import matplotlib.pyplot as plt

X = []
Y = []
Z = []

stick_length_pub = rospy.Publisher('/stick_length', Int32)

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
        i = 0
        r = rospy.Rate(1000)
        while (1):
            self.key = getKey()
            # 显示Opencv格式的图像
            if self.img.any():
                lower = np.array([0, 0, 0])
                upper = np.array([100, 200, 60])
            

                # 根据颜色空间阈值生成掩膜
                self.img_bin = cv2.inRange(self.img, lower, upper)
                # cv2.imshow("img", img)
                contours, _ = cv2.findContours(self.img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                rects = []
                for contour in contours:
                    x,y,w,h = cv2.boundingRect(contour)
                   
                    area = cv2.contourArea(contour)
                    if (area > 400):
                        rects.append([x,y,w,h])
                img_rects = draw_rects_on_img(self.img, rects)
                cv2.imshow("image with rects", img_rects)
                stick_length_pub.publish(h)


            cv2.waitKey(3)

            if (self.key == 'r'):
                cv2.imwrite(self.pwd + "stick.jpg", self.img)
            if (self.key == '\x03' or i > 500):
                break
            r.sleep()


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    # input = input("Do you want to record video or capture image? (0 for video and 1 for picture)")

    monitor = Monitor()
    monitor.main_loop()

