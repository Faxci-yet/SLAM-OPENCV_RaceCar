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
