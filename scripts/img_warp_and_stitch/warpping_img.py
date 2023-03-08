#!/usr/bin/env python

import cv2
import numpy as np
from numpy.linalg import inv,pinv
import math
import sys
# vector<double> camera = { 370.0, 0, 640, 0, 375, 360, 0, 0, 1 };
K = np.array([[370.0, 0, 640], [0, 375, 360], [0, 0, 1]])

def get_H(Rx_angle):
    global K
    alpha = Rx_angle*math.pi/180
    R = np.array([ [1, 0, 0], [0, math.cos(alpha), -math.sin(alpha)], [0, math.sin(alpha), math.cos(alpha)] ]) # x
    # R = np.array([ [math.cos(alpha), 0, math.sin(alpha)], [0, 1, 0],[-math.sin(alpha), 0, math.cos(alpha)] ]) # y
    # R = np.array([ [math.cos(alpha), -math.sin(alpha), 0],[math.sin(alpha), math.cos(alpha), 0], [0, 0, 1] ]) # z
    H = np.matmul(K, R)
    H = np.matmul(H, inv(K)) 
    return H 

if __name__ == '__main__' :
    # Read source image.
    pwd = sys.path[0]
    front = cv2.imread(pwd+'/pics_ocean/front10.jpg')
    down = cv2.imread(pwd+'/pics_ocean/down10.jpg')

    # cv2.imshow("front", front)
    # cv2.imshow("down", down)
    size = (down.shape[1],down.shape[0])

    print(size)

    H_front = get_H(45)
    H_down = get_H(-45)
    
    # Warp source image to destination based on homography
    front_out = cv2.warpPerspective(front, H_front, size)[0:350, 180:1100]  # 横轴x, 纵轴y 350*920
    down_out = cv2.warpPerspective(down, H_down, size)[370:1280, 180:1100] # 910*920
    # print(type(front_out))
    # Display images
    # cv2.imshow("Source Image", front)
    # cv2.imshow("Destination Image", im_dst)
    cv2.imshow("Warped front_out", front_out)
    cv2.imshow("Warped down_out", down_out)
    cv2.imwrite("front_out.jpg", front_out)
    cv2.imwrite("down_out.jpg", down_out)
    cv2.waitKey(0)

