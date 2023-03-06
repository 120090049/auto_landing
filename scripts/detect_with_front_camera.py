import sys
import numpy as np 
import cv2
from skimage import feature as ft 
from sklearn.externals import joblib
import os
import time

SHOW_PROCESS = True
RECORD_AS_VIDEO = True

cls_names = ["diagonal", "front_rear", "left_right", "background"]
img_label = {"diagonal": 0, "front_rear": 1, "left_right": 2, "background": 3}

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

class Detect():
    def __init__(self, row, col, clf, preprocess_res, detected_res, cv_or_sk = 1):
        self.image = np.ones(shape=(row, col, 3), dtype=int)
        self.rows = row # 720
        self.cols = col # 1280
   
        self.lower = np.array([50, 0, 0])
        self.upper = np.array([179, 255, 115])

        self.kernelDilation = np.ones((3,3), np.uint8) 
        self.kernelOpen = np.ones((4,4), np.uint8) 
        self.kernelClose = np.ones((4,4), np.uint8) 
        
        self.clf = clf

        self.cv_or_sk = cv_or_sk

        self.preprocess_res = preprocess_res
        self.detected_res = detected_res

    def preprocess_img(self, erode_dilate=True):
        """preprocess the image for contour detection.
        Args:
            erode_dilate: erode and dilate or not.
        Return:
            img_bin: a binary image (blue and red).

        """
        imgHSV = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        # 颜色空间阈值
        # 根据颜色空间阈值生成掩膜
        img_bin = cv2.inRange(imgHSV, self.lower, self.upper)
        

        img_bin = cv2.dilate(img_bin, self.kernelDilation, iterations=1)
        img_bin = cv2.morphologyEx(img_bin, cv2.MORPH_CLOSE, self.kernelClose)
        img_bin = cv2.morphologyEx(img_bin, cv2.MORPH_OPEN, self.kernelOpen)
        contours, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        rects = []
        for contour in contours:
            x,y,w,h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            if (w/h > 1 and w/h < 4.5 and area > 400):
                rects.append([x,y,w,h])
        return rects


    def hog_extra_and_svm_class(self, target_area, resize = (64, 32)):
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
        rects = self.preprocess_img(erode_dilate = False)
            
        if (SHOW_PROCESS):
            img_rects = draw_rects_on_img(self.image, rects)
            cv2.imshow("image with rects", img_rects)
            if (RECORD_AS_VIDEO):
                self.preprocess_res.write(img_rects)
        # if (len(rects) == 0):
        #     return True
        # return False
        
        target = 0
        for rect in rects: # rectangle(x1 0, y1 1) (w 2, h 3) 
            # xc = int(rect[0] + rect[2]/2) 
            # yc = int(rect[1] + rect[3]/2)

            # size = max(rect[2], rect[3])
            x1 = max(0, rect[0]-10)
            y1 = max(0, rect[1]-5)
            x2 = min(self.cols, rect[0]+rect[2]+10)
            y2 = min(self.rows, rect[1]+rect[3]+5)
            
            target_area = img[y1:y2, x1:x2]
            target_area = cv2.resize(target_area, (64, 32))
           
            prob_for_class = self.hog_extra_and_svm_class(target_area)
            
            cls_num = np.argmax(prob_for_class[0:-1])
            # cls_name = cls_names[cls_num]
            prop = prob_for_class[cls_num]
        
            if (prob_for_class[-1] < 0.4):
                center = int((x1 + x2)/2)
                target = center - self.cols/2
                cv2.rectangle(self.image,(x1, y1), (x2,y2), (0,0,255), 2)
                cv2.putText(self.image, "boat"+str(prop), (x1, y1), 1, 1.5, (0,0,255),2)
                cv2.putText(self.image, str(target), (center, y2), 1, 1.5, (0,0,255),2)
        
        if (RECORD_AS_VIDEO):
            self.detected_res.write(self.image)
        cv2.imshow("detect result", self.image)
        return target


if __name__ == "__main__":

    pwd = sys.path[0]  #/media/clp/Data1/ECE4513/traffic_sign_detection
    
    cap = cv2.VideoCapture(pwd + "/data/videos/test.mp4")
    row = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    col = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    
    clf = joblib.load(pwd+"/svm_parameter/my_svm_model.pkl")
    preprocess_res = cv2.VideoWriter(pwd+'/result_video/preprocess_res.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 10, (1280,720))
    detected_res = cv2.VideoWriter(pwd+'/result_video/detected_res.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 10, (1280,720))
    
    Detector = Detect(row, col, clf, preprocess_res, detected_res)

    # ret, img = cap.read()
    # tag = Detector.process(img)
    # cv2.waitKey(0)
    i = 0
    average_time = 0
    while (1):
        i += 1
        # start = time.time()
        ret, img = cap.read()
        target = Detector.process(img)
        # end = time.time()
        # duration = end-start
        # average_time = (average_time*(i-1) + duration)/i
        # print("average_time = ", average_time)
        print(target)
        
        cv2.waitKey(10)
    
    # # cv2.imwrite("detect_result.jpg", img_bbx)
