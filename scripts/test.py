import sys
import cv2
from img_warp_and_stitch.stitch_stream import MATCHING

if __name__ == "__main__":
    pwd = sys.path[0]
    cap_front = cv2.VideoCapture(pwd+'/img_warp_and_stitch/videos/front.mp4')
    cap_down = cv2.VideoCapture(pwd+'/img_warp_and_stitch/videos/down.mp4')

    img_stitch = MATCHING(record=True)
    while (1):
        ret, img1 = cap_front.read()
        ret, img2 = cap_down.read()
        img_stitch.process_img(img1, img2)
        cv2.waitKey(40)