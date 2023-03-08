#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#author : bingo
#email  : bingobin.lw@gmail.com
import sys
import math
import numpy as np
from skimage import feature as ft
from sklearn.externals import joblib
import rospy
from geometry_msgs.msg import Twist
import sys

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from prometheus_msgs.msg import DetectionInfo
from img_warp_and_stitch.stitch_stream import MATCHING


SHOW_PROCESS = False
RECORD_AS_VIDEO = False

cls_names = ["diagonal", "front_rear", "left_right", "background"]
img_label = {"diagonal": 0, "front_rear": 1, "left_right": 2, "background": 3}

# 1280*720
ROW = 720
COL = 1280

# 680*720
STITCH_ROW = 690
STITCH_COL = 680

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
        cv2.rectangle(img_copy, (x, y), (x+w, y+h), (0, 255, 0), 2)
    return img_copy


class Detect():
    def __init__(self, row, col, clf, cv_or_sk=1):
        self.image = np.ones(shape=(row, col, 3), dtype=int)
        self.rows = row  # 720
        self.cols = col  # 1280
        self.img_title = str(row) + "*" + str(col)
        self.lower = np.array([50, 0, 0])
        self.upper = np.array([179, 255, 115])

        self.kernelDilation = np.ones((3, 3), np.uint8)
        self.kernelOpen = np.ones((4, 4), np.uint8)
        self.kernelClose = np.ones((4, 4), np.uint8)

        self.clf = clf

        self.cv_or_sk = cv_or_sk


    def preprocess_img(self, erode_dilate=True):
        """preprocess the image for contour detection.
        Args:
            erode_dilate: erode and dilate or not.
        Return:
            img_bin: a binary image (blue and red).

        """
        imgHSV = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
     
        img_bin = cv2.inRange(imgHSV, self.lower, self.upper)

        img_bin = cv2.dilate(img_bin, self.kernelDilation, iterations=1)
        img_bin = cv2.morphologyEx(img_bin, cv2.MORPH_CLOSE, self.kernelClose)
        img_bin = cv2.morphologyEx(img_bin, cv2.MORPH_OPEN, self.kernelOpen)
        contours, _ = cv2.findContours(
            img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        rects = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            if (w/h > 1 and w/h < 4.5 and area > 400):
                rects.append([x, y, w, h])
        return rects

    def hog_extra_and_svm_class(self, target_area, resize=(64, 32)):
        """classify the region target_area.
        Args:
            target_area: region target_area (numpy array).
            clf: a SVM model.
            resize: resize the region target_area
                (default: (64, 64))
        Return:
            prob_for_class: propabality of all classes.
        """
        img = cv2.cvtColor(target_area, cv2.COLOR_BGR2GRAY)
        img = cv2.resize(img, resize)
        bins = 9
        cell_size = (8, 8)
        cpb = (2, 2)
        norm = "L2"

        # if (self.cv_or_sk):
        #     hog = cv2.HOGDescriptor(resize, (16,16), (8,8), (8,8), 9)
        #     features = hog.compute(img).reshape((1, -1))
        # else:
        features = ft.hog(img, orientations=bins, pixels_per_cell=cell_size,
                          cells_per_block=cpb, block_norm=norm, transform_sqrt=True).reshape((1, -1))

        # print ("cvfeature = ", features.shape)

        prob_for_class = self.clf.predict_proba(features)[0]
        prob_for_class = np.round(prob_for_class, 2)
        return prob_for_class

    def process(self, img):
        self.image = img
        rects = self.preprocess_img(erode_dilate=False)

        if (SHOW_PROCESS):
            img_rects = draw_rects_on_img(self.image, rects)
            cv2.imshow("image with rects", img_rects)
           
        target_yaw = 0
        detected = False
        for rect in rects:  # rectangle(x1 0, y1 1) (w 2, h 3)

            x1 = max(0, rect[0]-10)
            y1 = max(0, rect[1]-5)
            x2 = min(self.cols, rect[0]+rect[2]+10)
            y2 = min(self.rows, rect[1]+rect[3]+5)

            target_area = img[y1:y2, x1:x2]
            target_area = cv2.resize(target_area, (64, 32))

            prob_for_class = self.hog_extra_and_svm_class(target_area)

            cls_num = np.argmax(prob_for_class[0:-1])
            prop = prob_for_class[cls_num]

            if (prob_for_class[-1] < 0.6):
                center = int((x1 + x2)/2)
                target_yaw = 200 * (center - self.cols/2) / self.cols
                detected = True
                cv2.rectangle(self.image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(self.image, "boat"+str(prop),
                            (x1, y1), 1, 1.5, (0, 0, 255), 2)
                cv2.putText(self.image, "target_yaw(degree): "+str(target_yaw),
                            (center, y2), 1, 1.5, (0, 0, 255), 2)

        cv2.putText(self.image, "Detected: "+str(detected),
                    (1050, 40), 1, 1.5, (0, 0, 255), 2)
        
        cv2.imshow(self.img_title, self.image)
        return detected, target_yaw


class Front_camera_detect:
    def __init__(self):

        rospy.init_node('front_camera_detector', anonymous=True)

        rospy.Subscriber("/prometheus/sensor/monocular_front/image_raw", Image, self.front_camera_callback)
        rospy.Subscriber("/prometheus/sensor/monocular_down/image_raw", Image, self.down_camera_callback)
        rospy.Subscriber("/prometheus/switch/boat_det", Bool, self.switch_callback)

        self.boat_yaw_pub = rospy.Publisher('/prometheus/object_detection/boat_det', DetectionInfo, queue_size=10)
        self.boat_yaw_msg = DetectionInfo()
        self.boat_yaw_msg.detected = False

        self.bridge = CvBridge()
        self.switch = False
        self.pre_switch_state = False
        self.cv_image_front = np.ones(shape=(ROW, COL, 3), dtype=int)
        self.cv_image_down = np.ones(shape=(COL, COL, 3), dtype=int)

        pwd = sys.path[0]
        print(pwd + "/svm_parameter/my_svm_model.pkl")
        self.clf = joblib.load(pwd+"/svm_parameter/my_svm_model.pkl")
        self.Detector = Detect(ROW, COL, self.clf)
        
        self.Detector_sti = Detect(STITCH_ROW, STITCH_COL, self.clf)
        self.matcher = MATCHING(record=True)

    def front_camera_callback(self, msg):
        if (self.switch):
            try:
                # cv_image_front = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
                # 获取图像数据
                img_data = msg.data
                # 将图像数据转换为numpy数组
                img_np = np.frombuffer(img_data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
                # 将numpy数组转换为OpenCV格式的图像
                self.cv_image_front = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
                # 显示图像
                # cv2.imshow("Image from rostopic",self.cv_image_front)

            except CvBridgeError as e:
                print(e)
            cv2.waitKey(3)
        return

    def down_camera_callback(self, msg):
        if (self.switch):
            try:
                # cv_image_front = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
                # 获取图像数据
                img_data = msg.data
                # 将图像数据转换为numpy数组
                img_np = np.frombuffer(img_data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
                # 将numpy数组转换为OpenCV格式的图像
                self.cv_image_down = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
                # 显示图像
                # cv2.imshow("Image from rostopic",self.cv_image_front)

            except CvBridgeError as e:
                print(e)
            cv2.waitKey(3)
        return

    def switch_callback(self, switch_msg):
        self.switch = switch_msg.data
        if (self.pre_switch_state != self.switch):
            if (self.switch):
                print("front_camera_detector node is ON")
            else:
                print("front_camera_detector node is OFF")
        self.pre_switch_state = self.switch
        return
    

    # def main_loop(self):
    #     pwd = sys.path[0]
    #     cap_front = cv2.VideoCapture(pwd+'/videos/front.mp4')
    #     cap_down = cv2.VideoCapture(pwd+'/videos/down.mp4')

    #     rate = rospy.Rate(10) # 10hz
    #     while not rospy.is_shutdown():
    #         ret, img1 = cap_front.read()
    #         ret, img2 = cap_down.read()
    #         cv2.imshow("stitch_res", img1)
    #         if False:
    #             detected, target_yaw = self.Detector.process(img1)
    #             if not detected:
    #                 stitch_res = self.matcher.process_img(img1, img2)
    #                 if (stitch_res is not None):
    #                     # print(stitch_res.shape[:2])
    #                     detected, target_yaw = self.Detector_sti.process(stitch_res)
    #             # print(detected, target_yaw)
    #             self.boat_yaw_msg.header.stamp = rospy.Time.now()
    #             self.boat_yaw_msg.detected = detected
    #             self.boat_yaw_msg.yaw_error = math.radians(target_yaw)
    #             self.boat_yaw_pub.publish(self.boat_yaw_msg)
    #         rate.sleep()
    #     return

    def main_loop(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.switch:
                detected, target_yaw = self.Detector.process(self.cv_image_front)
                # if not detected:
                if True:
                    stitch_res = self.matcher.process_img(self.cv_image_front, self.cv_image_down)
                    if (stitch_res is not None):
                        # cv2.imshow("stitch_res", stitch_res)
                        # print(stitch_res.shape[:2])
                        detected, target_yaw = self.Detector_sti.process(stitch_res)
                # print(detected, target_yaw)
                self.boat_yaw_msg.header.stamp = rospy.Time.now()
                self.boat_yaw_msg.detected = detected
                self.boat_yaw_msg.yaw_error = math.radians(target_yaw)
                self.boat_yaw_pub.publish(self.boat_yaw_msg)
            rate.sleep()
        return


if __name__ == "__main__":

    front_camera_detector = Front_camera_detect()
    front_camera_detector.main_loop()
    # # cv2.imwrite("detect_result.jpg", img_bbx)
