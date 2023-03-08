import numpy as np
import cv2
import math, sys
from numpy.linalg import inv

# intrisic matirix
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

class MATCHING():
    def __init__(self, record=False):
        self.front = None
        self.down = None
        self.front_dia = None
        self.down_dia = None
        self.H_front = get_H(45)
        self.H_down = get_H(-45)
        self.H = np.array([[ 1.24350036e+00, -3.74674027e-01, -8.51797848e+01],
        [ 1.86412262e-03,  1.63740481e+00, -4.32051077e+02],
        [ 1.16233421e-04, -8.00900929e-04,  1.00000000e+00]])

        

        self.size_front = None
        self.size_down = None
        #输出图像的尺寸
        self.ww = 0
        self.hh = 0

        self.first_time = True
        self.transform_dist = None
        self.transform_arary = None
        
        
        self.record = record
        self.res_video = None
        if (self.record):
            pwd = sys.path[0]
            print("Record is saved at "+pwd+'/stitch_res.mp4')
            print( (self.ww,self.hh))
            self.res_video = cv2.VideoWriter(pwd+'/stitch_res.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 10, (self.ww,self.hh))
        

    def get_img(self, front_img, down_img):
        self.front = front_img
        self.down = down_img

    def warpping(self):
        self.front_dia = cv2.warpPerspective(self.front, self.H_front, self.size_front)[0:350, 180:1100]  # 横轴x, 纵轴y
        self.down_dia = cv2.warpPerspective(self.down, self.H_down, self.size_down)[370:1280, 180:1100]
        cv2.imshow("self.front_dia", self.front_dia)
        # cv2.imshow("self.down_dia", self.down_dia)
    def stitch_test(self):
        self.size_front = (self.front.shape[1], self.front.shape[0])
        self.size_down = (self.down.shape[1], self.down.shape[0])
        h1,w1 = self.front_dia.shape[:2]
        h2,w2 = self.down_dia.shape[:2]
        #获取四个点的坐标，变换数据类型便于计算
        img1_dims = np.float32([[0,0],[0,h1],[w1,h1],[w1,0]]).reshape(-1,1,2)
        img2_dims = np.float32([[0,0],[0,h2],[w2,h2],[w2,0]]).reshape(-1,1,2)
        #获取根据单应性矩阵透视变换后的图像四点坐标
        img1_transform = cv2.perspectiveTransform(img1_dims,self.H)
 
        #合并矩阵  获取最大x和最小x，最大y和最小y  根据最大最小计算合并后图像的大小；
        # #计算方式： 最大-最小
        result_dims = np.concatenate((img2_dims,img1_transform),axis = 0)
        [x_min,y_min] = np.int32(result_dims.min(axis=0).ravel()-0.5)
        [x_max,y_max] = np.int32(result_dims.max(axis=0).ravel()+0.5)
        #平移距离
        self.transform_dist = [-x_min, -y_min]
        #齐次变换矩阵
        self.transform_arary = np.array([[1,0,self.transform_dist[0]],
                                    [0,1,self.transform_dist[1]],
                                    [0,0,1]])
         #输出图像的尺寸
        self.ww = x_max-x_min
        self.hh = y_max-y_min
        result_img = cv2.warpPerspective(self.front_dia,self.transform_arary.dot(self.H),(self.ww,self.hh))
        result_img[self.transform_dist[1]+10:self.transform_dist[1]+h2,
                self.transform_dist[0]:self.transform_dist[0]+w2] = self.down_dia[10:h2, 0:w2]
        # cv2.imshow("test", result_img)
        
        return 

    def stitch(self):
        h2,w2 = self.down_dia.shape[:2]
        result_img = cv2.warpPerspective(self.front_dia,self.transform_arary.dot(self.H),(self.ww,self.hh))
        result_img[self.transform_dist[1]+10:self.transform_dist[1]+h2,
                self.transform_dist[0]:self.transform_dist[0]+w2] = self.down_dia[10:h2, 0:w2]
        height, width = result_img.shape[:2]
        # print(height, width)
        result_img = result_img[100:1300, 300:1200]
        if (self.record):
            self.res_video.write(result_img)
        return result_img
    
    def process_img(self, front_img, down_img):
        self.get_img(front_img, down_img)
        self.warpping()
        if (self.first_time):
            self.stitch_test()
            self.first_time = False
            return None
        else:
            stitch_res = self.stitch()
            # cv2.imshow("result", stitch_res)
            return stitch_res


if __name__ == "__main__":
    pwd = sys.path[0]
    cap_front = cv2.VideoCapture(pwd+'/videos/front.mp4')
    cap_down = cv2.VideoCapture(pwd+'/videos/down.mp4')
    fps = int(cap_down.get(cv2.CAP_PROP_FPS))
    video_writer = cv2.VideoWriter(pwd+'/videos/stitch.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (900,1200))
    
    img_stitch = MATCHING(record=True)
    while (1):
        ret, img1 = cap_front.read()
        ret, img2 = cap_down.read()
        res = img_stitch.process_img(img1, img2)
        if (res is not None):
            video_writer.write(res)
            cv2.imshow("res", res)
        cv2.waitKey(1)