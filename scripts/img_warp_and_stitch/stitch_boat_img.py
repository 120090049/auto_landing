import cv2
import numpy as np
from numpy.linalg import inv,pinv
import math, sys

# target H
# [[ 9.81110373e-01 -3.90955027e-01  8.70012010e+00]
# [-7.66571874e-03  1.34562262e+00 -3.50346900e+02]
# [-5.40140789e-06 -1.06337218e-03  1.00000000e+00]]

def get_homo(img1,img2):
    #创建特征转换对象
    sift = cv2.SIFT_create()
    #获取特征点和描述子
    k1,d1 = sift.detectAndCompute(img1,None)
    k2,d2 = sift.detectAndCompute(img2,None)
    #创建特征匹配器
    bf = cv2.BFMatcher()
    #使用描述子进行一对多的描述子匹配
    maches = bf.knnMatch(d1,d2,k=2)
    #筛选有效的特征描述子存入数组中
    verify_matches = []
    for m1,m2 in maches:
        if m1.distance <0.8*m2.distance:
            verify_matches.append(m1)
    #单应性矩阵需要最低四个特征描述子坐标点进行计算，判断数组中是否有足够,这里设为6更加充足
    # cv2.drawKeypoints(img1, k1, img1, (0,255,255))
    # cv2.drawKeypoints(img2, k2 ,img2, (0,255,255))
    if len(verify_matches) > 6:
        #存放求取单应性矩阵所需的img1和img2的特征描述子坐标点
        img1_pts = []
        img2_pts = []
        i = 0
        for m in verify_matches:
        #通过使用循环获取img1和img2图像优化后描述子所对应的特征点
            img1_pts.append(k1[m.queryIdx].pt)
            img2_pts.append(k2[m.trainIdx].pt)
            one = tuple(k1[m.queryIdx].pt)
            one = (int(one[0]), int(one[1]))
            two = tuple(k2[m.trainIdx].pt)
            two = (int(two[0]), int(two[1]))
            if (i % 1 == 0):
                cv2.circle(img1, one, 1, (0,0,255), 2)
                cv2.circle(img2, two, 1, (0,0,255), 2)
                # print(m.queryIdx, " + ", m.trainIdx)
            i += 1
        # cv2.imshow('test',img1)
        # cv2.imshow('test2',img2)
        #得到的坐标是[(x1,y1),(x2,y2),....]
        #计算需要的坐标格式：[[x1,y1],[x2,y2],....]所以需要转化
        # img1_pts = [[np.float(k) for k in i] for i in img1_pts]
        # img2_pts = [[np.float(k) for k in i] for i in img2_pts]
        img1_pts = np.array([list(i) for i in img1_pts])
        img2_pts = np.array([list(i) for i in img2_pts])
        #计算单应性矩阵用来优化特征点
        H,mask = cv2.findHomography(img1_pts,img2_pts,cv2.RANSAC,5.0)
        return H
    else:
        print("error")

def stitch_image(front_dia,down_dia,H):
    #1、获得每张图片的四个角点
    #2、对图片进行变换，（单应性矩阵使）
    #3、创建大图，将两张图拼接到一起
    #4、将结果输出
    #获取原始图的高、宽
    h1,w1 = front_dia.shape[:2]
    h2,w2 = down_dia.shape[:2]
    #获取四个点的坐标，变换数据类型便于计算
    img1_dims = np.float32([[0,0],[0,h1],[w1,h1],[w1,0]]).reshape(-1,1,2)
    img2_dims = np.float32([[0,0],[0,h2],[w2,h2],[w2,0]]).reshape(-1,1,2)
    #获取根据单应性矩阵透视变换后的图像四点坐标
    img1_transform_dim = cv2.perspectiveTransform(img1_dims,H)

    #合并矩阵  获取最大x和最小x，最大y和最小y  根据最大最小计算合并后图像的大小；
    # #计算方式： 最大-最小
    result_dims = np.concatenate((img2_dims,img1_transform_dim),axis = 0)
    [x_min,y_min] = np.int32(result_dims.min(axis=0).ravel()-0.5)
    [x_max,y_max] = np.int32(result_dims.max(axis=0).ravel()+0.5)
    #平移距离
    transform_dist = [-x_min, -y_min]
    #齐次变换矩阵
    transform_arary = np.array([[1,0,transform_dist[0]],
                                [0,1,transform_dist[1]],
                                [0,0,1]])
        #输出图像的尺寸
    ww = x_max-x_min
    hh = y_max-y_min
    result_img = cv2.warpPerspective(front_dia,transform_arary.dot(H),(ww,hh))
    result_img[transform_dist[1]+10:transform_dist[1]+h2,
            transform_dist[0]:transform_dist[0]+w2] = down_dia[10:h2, 0:w2]

    #将img2添加进平移后的图像
    result_img[transform_dist[1]:transform_dist[1]+h2,
                transform_dist[0]:transform_dist[0]+w2] = down_dia
    return result_img


if __name__ == '__main__' :
    pwd = sys.path[0]
    img1 = cv2.imread(pwd+"/front_out.jpg")
    img2 = cv2.imread(pwd+"/down_out.jpg")
    # cv2.imshow("down",img2)
    if (img1 is None) or (img2 is None):
        print("路径问题")
    else:
        img1 = cv2.resize(img1,(920,350))
        img2 = cv2.resize(img2,(920,910))
        #把数据压到栈中
        inputs = np.vstack((img1,img2))
        # cv2.imshow("original img", inputs)

    H = get_homo(img1,img2)
    result_img = stitch_image(img1,img2,H)[100:1300, 300:1200]
    print(H)
    print(result_img.shape)
    cv2.imshow('result_img',result_img)
    cv2.imwrite("result_img.jpg", result_img)
    cv2.waitKey()
