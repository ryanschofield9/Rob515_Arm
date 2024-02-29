#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

'''
range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

__target_color = ('red',)
# 设置检测颜色 #set the detection color 
def setTargetColor(target_color):
    global __target_color

    #print("COLOR", target_color)
    __target_color = target_color
    return (True, ())

# 找出面积最大的轮廓 #Find the profile with the largest area 
# 参数为要比较的轮廓的列表 #The parameter is a list of profiles to be compared
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓 #Go through all the outlines 
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积 #Calculate the profile area 
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓 #The profile of the largest area is only valid if the area is greater than 300 to filter out interference 

def reset():
    global count
    global track
    global _stop
    global get_roi
    global first_move
    global center_list
    global __isRunning
    global detect_color
    global action_finish
    global start_pick_up
    global __target_color
    global start_count_t1
    
    count = 0
    _stop = False
    track = False
    get_roi = False
    center_list = []
    first_move = True
    __target_color = ()
    detect_color = 'None'
    action_finish = True
    start_pick_up = False
    start_count_t1 = True

for i in color_range:
    if i in __target_color:
                    detect_color = i
                    frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])  # 对原图像和掩模进行位运算 # Perform bit operations on the original image and mask
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算 #start the operation 
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算 #Closed operations
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓 #Find the outline 
                    areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓 #Find the maximum contour 
if area_max > 2500:  # 有找到最大面积 #There is the largest area found 
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))

            roi = getROI(box) #获取roi区域 #......
            get_roi = True

            img_centerx, img_centery = getCenter(rect, roi, size, square_length)  # 获取木块中心坐标 #Get the center coordinates of the block 
            world_x, world_y = convertCoordinate(img_centerx, img_centery, size) #转换为现实世界坐标 # ....
            
            
            cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
            cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1) #绘制中心点
            track = True

'''
class Perception():
    def __init__(self):
        self.count = 0
        self.stop = False
        self.square_length = 3
        self.roi = ()
        self.track = False
        self.get_roi = False
        self.center_list = []
        self.first_move = True
        self.target_color = ()
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True
        self.size = (640, 480)
        self.last_x = 0
        self.last_y = 0
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }
        self.color_range = {
            'red': [(0, 151, 100), (255, 255, 255)], 
            'green': [(0, 0, 0), (255, 115, 255)], 
            'blue': [(0, 0, 0), (255, 255, 110)], 
            'black': [(0, 0, 0), (56, 255, 255)], 
            'white': [(193, 0, 0), (255, 250, 255)], 
        }

    def set_target_color(self,color): #color is a value of 'red', 'blue', 'green', 'black' or 'white
        self.target_color = color 
        return self.target_color 
    
    def get_frame_LAB(self,img):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)#..... the area is detected until there is none 
        if self.get_roi and self.start_pick_up:
            self.get_roi = False
            frame_gb = getMaskROI(frame_gb, self.roi, self.size)    
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间 #Convert images to LAB space
        return frame_lab

    def find_contours (self, frame_lab):

        for i in self.color_range:
            if i in self.target_color:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])  # 对原图像和掩模进行位运算 # Perform bit operations on the original image and mask
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算 #start the operation 
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算 #Closed operations
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓 #Find the outline 
        return contours 
    
    def calculate_profile_area(self, contours):
        contour_area_holder = 0
        max_area = 0
        max_contour = None

        for c in contours:  # 历遍所有轮廓 #Go through all the outlines 
            contour_area_holder = math.fabs(cv2.contourArea(c))  # 计算轮廓面积 #Calculate the profile area 
            if contour_area_holder > max_area:
                max_area = contour_area_holder
                if contour_area_holder > 300:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                    max_contour = c
        return max_contour, max_area  # 返回最大的轮廓 #The profile of the largest area is only valid if the area is greater than 300 to filter out interference 
    
    def turn_into_box(self, max_contour):
        rect = cv2.minAreaRect(max_contour)
        box = np.int0(cv2.boxPoints(rect))
        return rect, box 
    
    def get_ROI(self,box):
        self.roi = getROI(box) #获取roi区域 #......
        self.get_roi = True
    
    def get_center(self, rect):
        img_center_x, img_center_y = getCenter(rect, self.roi, self.size, self.square_length)
        return img_center_x, img_center_y

    def get_in_world_frame(self, img_center_x, img_center_y):
        world_x, world_y = convertCoordinate(img_center_x, img_center_y, self.size)
        return world_x, world_y
    
    def draw_box(self,img, target_color, world_x, world_y, box):
        cv2.drawContours(img, [box], -1, self.range_rgb[target_color], 2)
        cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[target_color], 1) 
        self.track = True
        return img
    
    def run (self,color, run_time):
        target_color = self.set_target_color(color)
        my_camera = Camera.Camera()
        my_camera.camera_open()
        start = time.time()

        while (time.time()-start) < run_time:
            img = my_camera.frame
            if img is not None:
                frame = img.copy()
                frame_lab = self.get_frame_LAB(frame)
                contours = self.find_contours(frame_lab)
                max_contour, max_area = self.calculate_profile_area(contours)
                rect,box = self.turn_into_box(max_contour)
                self.get_ROI(box)
                img_center_x, img_center_y = self.get_center(rect)
                world_x, world_y = self.get_in_world_frame(img_center_x, img_center_y)
                Frame = self.draw_box(img,target_color, world_x, world_y, box)
                cv2.imshow('Frame', Frame)
                key = cv2.waitKey(1)
                if key == 27:
                    break
        my_camera.camera_close()
        cv2.destroyAllWindows()

    def get_coordinates(self, color):
        target_color = self.set_target_color(color)
        my_camera = Camera.Camera()
        my_camera.camera_open()
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            frame_lab = self.get_frame_LAB(frame)
            contours = self.find_contours(frame_lab)
            max_contour, max_area = self.calculate_profile_area(contours)
            rect,box = self.turn_into_box(max_contour)
            self.get_ROI(box)
            img_center_x, img_center_y = self.get_center(rect)
            world_x, world_y = self.get_in_world_frame(img_center_x, img_center_y)
            return (world_x, world_y)


if __name__ == '__main__':
    perception = Perception()
    perception.run('red',10)
    perception.run('blue', 10)
    perception.run('green',10)
    '''
    target_color = perception.set_target_color('red')
    my_camera = Camera.Camera()
    my_camera.camera_open()
    time.sleep(2)
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            frame_lab = perception.get_frame_LAB(frame)
            contours = perception.find_contours(frame_lab)
            max_contour, max_area = perception.calculate_profile_area(contours)
            rect,box = perception.turn_into_box(max_contour)
            perception.get_ROI()
            img_center_x, img_center_y = perception.get_center(rect)
            world_x, world_y = perception.get_in_world_frame(img_center_x, img_center_y)
            Frame = perception.draw_box(img,target_color, world_x, world_y)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
    '''
    
