'''
MIT License

Copyright (c) 2023 Faxci-yet

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''


import cv2
import numpy as np
from BarricadeView_forROS import BarricadeView
import rospy
from racecar.msg import Deviation


def main():
    rospy.init_node('camera_analys')
    cap = cv2.VideoCapture(0)
    cv2.namedWindow('camera', cv2.WINDOW_AUTOSIZE)

    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            if (frame is not None) and cv2.waitKey(1) != 27:
                bv = BarricadeView(frame)  # 创建一个bv实例
                bv.image_process()  # 图像处理
                bv.send_deviation()
                cv2.imshow('camera', bv.image)

            else:
                print("无画面")
                cv2.destroyALLWindows()

        # print('aa')
        else:
            print("无法读取摄像头！")

    cap.release()
    #cv2.waitKey(0)
    #cv2.destroyALLWindows()
    # image = cv2.imread('zhuitong3.png')  # 图片输入

    # start_time = time.time()
    # finish_time = time.time()
    # print('i used ', (finish_time - start_time), 's to do!')  # 计时语句
    # print('finish my work')


if __name__ == '__main__':
    main()
