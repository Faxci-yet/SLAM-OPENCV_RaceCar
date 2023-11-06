import cv2
import numpy as np
import time
import rospy
from std_msgs.msg import String
from racecar.msg import Deviation

class BarricadeView(object):
    '''
    author: Dong.ZB
    description:    一个识别锥桶的对象，
                    包含了图片预处理方法 image_pretreatment，
                    锥桶位置识别方法 get_center_by_color，
                    图片标记方法 image_draw
                    小车偏差计算方法
                    小车偏差发布方法
    workToBeDone:
    '''
    def __init__(self, image):
        self.ball_color = ['red', 'blue']
        self.color_dist = {'red': {'Lower':np.array([0, 60, 60]), 'Upper': np.array([6,255,255])},
                           'blue': {'Lower':np.array([100, 80, 46]), 'Upper': np.array([124,255,255])},
                           'green': {'Lower':np.array([35, 43, 35]), 'Upper': np.array([90,255,255])}, }

        self.topic_name = '/deviation'  # 偏差话题名称
        self.image = image

        self.gs_frame = []  # 预处理部分属性
        self.hsv = []
        self.erode_hsv = []
        self.image_center_x = 0
        self.image_center_y = 0

        self.red_candidates = []  # 锥桶位置信息属性
        self.blue_candidates = []
        self.red_center_Xs = []
        self.red_center_Ys = []
        self.blue_center_Xs = []
        self.blue_center_Ys = []
        self.deviation_for_send = 0
        self.deviations = []
        self.deviation = 0
        self.red_max_deviation = 0
        self.blue_max_deviation = 0
        self.red_max_index = 0
        self.blue_max_index = 0
        self.msgdev = Deviation()
        self.publisher = rospy.Publisher(self.topic_name, Deviation, queue_size=10)

        self.red_height_width_ratio = []
        self.blue_height_width_ratio = []
        self.red_center_Xmax = 0
        self.blue_center_Xmax = 0


    def image_pretreatment(self):
        '''
        图像预处理方法
        包含了高斯模糊，bgr2hsv，腐蚀，计算图像中心点
        :return: none
        '''
        self.gs_frame = cv2.GaussianBlur(self.image, (5, 5), 1)  # 高斯模糊
        self.hsv = cv2.cvtColor(self.gs_frame, cv2.COLOR_BGR2HSV)  # 转化成HSV图像
        self.erode_hsv = cv2.erode(self.hsv, None, iterations=1)  # 腐蚀 粗的变细
        self.image_center_x = int((len(self.image[0])) / 2) #计算中心点
        self.image_center_y = int((len(self.image)) / 2)

    def get_contour_centers(self,contours):
        """
           Calculate the centers of the contours
           :param contours: Contours detected with find_contours
           :return: object centers as numpy array
        """
        centers = []
        angles = []
        rects = []

        height_width_ratio = []

        for cnt in contours:
            rect = cv2.minAreaRect(cnt)

            # print("中心坐标：", rect[0])
            #     # print("宽度：", rect[1][0])
            #     # print("长度：", rect[1][1])
            #     # print("旋转角度：", rect[2])

            w = rect[1][0]
            h = rect[1][1]
            x = rect[0][0]
            y = rect[0][1]
            if w >= h:
                angle = rect[2]
            if w < h:
                angle = rect[2] - 90
            angles.append(angle)
            centers.append([x, y])
            rects.append(rect)
        # 返回的是长边和x轴夹角
        return centers, angles, rects

    def get_center_by_color(self,color):

        area_rects = []
        candidates = []
        index = 0
        centers = []
        centers_X = []
        centers_Y = []
        centers_XY = []

        height_width_ratio = []
        centers_Xmax = 0

        inRange_hsv = cv2.inRange(self.erode_hsv, self.color_dist[color]['Lower'], self.color_dist[color]['Upper'])
        contours, hierarchy = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centers, angles, rects = self.get_contour_centers(contours)
        hierarchy = hierarchy[0]

        for component in zip(contours, hierarchy):
            contour = component[0]
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.1 * peri, True)
            area = cv2.contourArea(contour)
            # print('area = ',area)

            M = cv2.moments(contour)

            if M["m00"]:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX = None
                cY = None
            if area >= 5000:
                tmp = {'index': index, 'cx': cX, 'cy': cY, 'contour': contour}
                # print(index)
                candidates.append(contour)
                centers_X.append(cX)
                centers_Y.append(cY)
                centers_XY.append(cX)
                centers_XY.append(cY)
                if abs(cX - self.image_center_x) > centers_Xmax:
                    centers_Xmax = abs(cX - self.image_center_x)
                    # centers_Xmax = cX - self.image_center_x
                index += 1

        return candidates, centers_X, centers_Y, height_width_ratio, centers_Xmax

    def image_draw(self,color):

        '''
        在原始图片上进行标注，锥桶识别结果可视化
        :param color:’red‘, 'blue'
        :return: none
        '''

        area_rects = []
        boxs = []
        if color == 'red':
            cv2.line(self.image, (self.image_center_x, 0), (self.image_center_x, len(self.image)), (255, 255, 50), 10)
            for candidate in self.red_candidates:
                rect = cv2.minAreaRect(candidate)
                area_rects.append(rect)

            for area_rect in area_rects:
                box = cv2.boxPoints(area_rect)
                boxs.append(box)

            for box in boxs:
                cv2.drawContours(self.image, [np.int0(box)], -1, (0, 0, 255), 2)

            # print('len of red_center_Xs = ', len(self.red_center_Xs))
            # for i in range(len(self.red_center_Xs)):
            #     cv2.line(self.image, (self.red_center_Xs[i], self.red_center_Ys[i]),
            #              (self.image_center_x, self.red_center_Ys[i]),
            #              (0, 255, 0), 5)
            #     i += 1
            if self.red_center_Xs is not None:
            	self.red_center_Xmax = min(self.red_center_Xs)
            	cv2.line(self.image, (self.red_center_Xmax, 0),
               	          (self.red_center_Xmax, len(self.image)),
               	          (0, 255, 0), 5)
            	print(self.red_center_Xmax)

        elif color == 'blue':
            for candidate in self.blue_candidates:
                rect = cv2.minAreaRect(candidate)
                area_rects.append(rect)

            for area_rect in area_rects:
                box = cv2.boxPoints(area_rect)
                boxs.append(box)

            for box in boxs:
                cv2.drawContours(self.image, [np.int0(box)], -1, (255, 0, 0), 2)

            # for i in range(len(self.blue_center_Ys)):
            #     cv2.line(self.image, (self.red_center_Xs[i], self.red_center_Ys[i]),
            #              (self.image_center_x, self.red_center_Ys[i]),
            #              (0, 255, 0), 5)
            #     i += 1
            if self.blue_center_Xs:
            	self.blue_center_Xmax = max(self.blue_center_Xs)
            	cv2.line(self.image, (self.blue_center_Xmax, 0),
               	          (self.blue_center_Xmax, len(self.image)),
               	          (0, 255, 0), 5)
            	print(self.blue_center_Xmax)
            else:
            	print('blue none')
            	
           

    def image_display(self):
        cv2.imshow('image', self.image)
        cv2.waitKey(1)

    def sum_deviation(self):
        sum_of_deviation_left = 0
        sum_of_deviation_right = 0
        red_deviations = []
        blue_deviations = []
        # self.red_max_deviation = 0
        # self.blue_max_deviation = 0
        deviation_all = 0
        for center_x in self.red_center_Xs:
            deviation_left = abs(self.image_center_x - center_x)
            red_deviations.append(deviation_left)
            sum_of_deviation_left += deviation_left
        for center_x in self.blue_center_Xs:
            deviation_right = abs(self.image_center_x - center_x)
            blue_deviations.append(deviation_right)
            sum_of_deviation_right += deviation_right
        self.red_max_deviation = max(red_deviations)
        self.blue_max_deviation = max(blue_deviations)
        self.red_max_index = red_deviations.index(self.red_max_deviation)
        self.blue_max_index = blue_deviations.index(self.blue_max_deviation)
        deviation_all = self.blue_max_deviation - self.red_max_deviation
        print('deviation = ', deviation_all)

    def get_max_value(self,lists):
        red_deviations = []
        blue_deviations = []
        if self.red_center_Xs is not None:
            for center_x in self.red_center_Xs:
                red_deviation = abs(self.image_center_x - center_x)
                red_deviations.append(red_deviation)
            self.red_max_deviation = max(red_deviations)
        else:
            self.red_max_deviation = abs(self.image_center_x - 0)


    def max_deviation(self):
        deviation = 0
        right = abs(self.image_center_x - self.blue_center_Xmax)
        left = abs(self.image_center_x - self.red_center_Xmax) 
        deviation = right - left
        print('deviation = ', deviation)
        return deviation

    def avarage_calculation(self,centers_Y):
        sum_of_centers = 0
        avarage_Y = 0
        if centers_Y:
            for center_Y in centers_Y:
                sum_of_centers += center_Y
            avarage_Y = sum_of_centers//len(centers_Y)
            return avarage_Y
        else:
            return 0

    def avarage_deviation(self):
        deviaiton = 0
        deviation_left = 0
        deviation_right = 0
        red_avarage_Y = 0
        blue_avarage_Y = 0
        red_avarage_Y = self.avarage_calculation(self.red_center_Ys)
        blue_avarage_Y = self.avarage_calculation(self.blue_center_Ys)
        deviation_left = abs(self.image_center_y - red_avarage_Y)
        deviation_right = abs(self.image_center_y - blue_avarage_Y)
        deviaiton = deviation_right - deviation_right
        print('deviation = ', deviaiton)
        return deviaiton

    def send_deviation(self):
        self.deviation = self.max_deviation()
        self.msgdev.deviation = self.deviation
        self.publisher.publish(self.msgdev)

    def image_process(self):
        self.image_pretreatment()
        self.red_candidates, self.red_center_Xs, self.red_center_Ys, self.red_height_width_ratio, self.red_center_Xmax \
            = self.get_center_by_color('red')
        self.blue_candidates, self.blue_center_Xs, self.blue_center_Ys, self.red_height_width_ratio, self.red_center_Xmax \
            = self.get_center_by_color('blue')
        self.image_draw('red')
        self.image_draw('blue')
        # cv2.imshow('image', self.image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()





