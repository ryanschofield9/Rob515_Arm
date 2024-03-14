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
from perception import Perception
import math 
import time 

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

class Motion():
    def __init__(self):
        self.AK = ArmIK()
        self.perception = Perception()
        self.detected_object = False
        self.count_second = 0
        self.first_move = True
        self.action_finish = True
        self.start_pick_up = False
        self.servo_1 = 500
        self.rotation_angle = 0 
        self.coordinate = {
        'red':   (-15 + 0.5, 12 - 0.5, 1.5),
        'green': (-15 + 0.5, 6 - 0.5,  1.5),
        'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        }

    def go_to_location (self,coordinates,alpha, alpha1, alpha2, movetime = None ):
            result = self.AK.setPitchRangeMoving(coordinates, alpha, alpha1, alpha2, movetime)
            return result
    
    def detect_object(self, color, my_camera):
        world_x, world_y = self.perception.get_coordinates(color, my_camera)
        return (world_x, world_y)

    def first_to_object (self, color, my_camera):
        #put in full code to run this if its the first move 
        while not self.detected_object: 
            self.world_x, self.world_y = self.detect_object(color, my_camera)
            self.detected_object = True 
        result = self.go_to_location((self.world_x, self.world_y-2, 12), -90, -90, 0)
        if result == False:
            self.unreachable = True  
        else: 
            self.unreachable = False
        time.sleep(result[2]/1000) #wait a little while 
        self.start_pick_up = False 
        self.first_move = False
        self.action_finished = True 
    
    def second_to_object(self, color, my_camera):
        #if not first_move and not unreachable
        #do this while self.start_pick_up is true
        self.action_finished = False 
        world_x, world_y = self.detect_object(color, my_camera)
        if self.count_second < 3:
            if math.isclose(world_x, self.world_x, rel_tol = 1) and math.isclose(world_y, self.world_y, rel_tol =1):
                self.start_pick_up = True
            else: 
                self.count_second = self.count_second +1 
            
        else: 
            self.world_x, self.world_y = self.detect_object(color, my_camera)
            self.count_second = 0 
        
    def grippers(self, open):
        if open:
            Board.setBusServoPulse(1, self.servo_1 - 310, 500) #open gripper 
            time.sleep(1)
        else: 
            Board.setBusServoPulse(1, self.servo_1+800, 500)  #close gripper 
            time.sleep(1)
    
    def rotate_gripper(self, color = None):
        if color == None:
            servo2_angle = getAngle(self.world_x, self.world_y, self.rotation_angle) # get roation angle 
        else: 
            servo2_angle = getAngle(self.coordinate[color][0], self.coordinate[color][1], -90)
        
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(1) 
    
    def lower_block(self, color=None):
        if color == None:
            self.go_to_location((self.world_x, self.world_y, 9), 0, -90, 0, 1000)    
        else: 
            self.go_to_location((self.coordinate[color][0], self.coordinate[color][1], self.coordinate[color][2] + 3), -90, -90, 0)
            time.sleep(0.5)
            self.go_to_location((self.coordinate[color]), -90, -90, 0, 1000)

        time.sleep(2)

    def raise_block(self, color = None):
        if color == None:
            Board.setBusServoPulse(2, 500, 500)
            self.go_to_location((self.world_x, self.world_y, 15 ), -90, -90, 0, 1000)
        else:
            self.go_to_location((self.coordinate[color][0], self.coordinate[color][1], 12), -90, -90, 0)
        time.sleep(1)
    
    def starting_position(self):
        Board.setBusServoPulse(1, self.servo_1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.go_to_location((0, 10, 10), -30, -30, -90, 1500)
    
    def reset(self):
        self.detected_object = False
        self.count_second = 0
        self.first_move = True
        self.action_finish = True
        self.start_pick_up = False
    

    def run (self, color, my_camera):
        self.starting_position()
        self.perception.run(color,5,my_camera)
        if self.first_move:
            self.first_to_object(color, my_camera) # Go close to the location of the found block 
        if not self.first_move and not self.unreachable:
            while(self.start_pick_up ==False): 
                self.second_to_object(color, my_camera) # make sure the object hasn't moved in a while
            self.grippers(True) # open grippers 
            self.rotate_gripper() #calculate needed angle of the gripper and rotate to that angle 
            self.fix_offset()
            self.lower_block() #move directly to block location and lower 
            self.grippers(False) #close grippers 
            self.raise_block() # raise block up 
            '''
            self.raise_block(color) # go close to predetermined location of block drop off 
            self.rotate_gripper(color) #calculate needed angle of gripper and rotate to that angle 
            self.lower_block(color) #slowly lower the block 
            self.grippers(True) #open grippers and drop block 
            self.starting_position() # go to starting position 
            '''
            self.reset()
            return True 
        elif self.unreachable: 
            print("Unreachable")
            self.reset()
            return False 
        else: 
            self.reset()
            return False 
        
    def fix_offset(self):
        self.world_x= self.world_x   
        self.world_y = self.world_y  

class Circle():
    def __init__(self):
        self.AK = ArmIK()
        self.perception = Perception()
        #self.detected_object = False
        #self.count_second = 0
        #self.first_move = True
        #self.action_finish = True
        #self.start_pick_up = False
        self.servo_1 = 500
        self.rotation_angle = 0 
        self.count_line = 0
        self.coordinate = {
        'red':   (-15 + 0.5, 12 - 0.5, 1.5),
        'green': (-15 + 0.5, 6 - 0.5,  1.5),
        'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        }
    
    def go_to_location (self,coordinates,alpha, alpha1, alpha2, movetime = None ):
            result = self.AK.setPitchRangeMoving(coordinates, alpha, alpha1, alpha2, movetime)
            return result
    
    def starting_position(self):
        Board.setBusServoPulse(1, self.servo_1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.go_to_location((0, 10, 10), -30, -30, -90, 1500)
    
    def rotate_gripper(self, x, y):
        servo2_angle = getAngle(x, y, self.rotation_angle) # get roation angle 
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(1) 

    def grippers(self, open):
        if open:
            Board.setBusServoPulse(1, self.servo_1 - 310, 500) #open gripper 
            time.sleep(1)
        else: 
            Board.setBusServoPulse(1, self.servo_1+900, 500)  #close gripper 
            time.sleep(1)
    
    def get_circle_pts (self, center, r):
        pts = []
        center_x = center[0]
        center_y = center[1]
        for i in range (24):
            x = center_x + r*math.cos(i*-15*(math.pi/180))
            y = center_y + r*math.sin(i *-15*(math.pi/180))
            z = 8
            pts.append((x,y,z))
        return pts 

    def run(self):
        print("go to starting pos")
        #self.starting_position()
        self.grippers(False)
        print("at starting pos")
        #print("go to 0.53,20.23,7")
        print("getting circle pts")
        pts = circle.get_circle_pts((0,29,5), 6)
        #self.rotate_gripper(pts[0][0], pts[0][1])
        result = self.go_to_location((pts[0][0], pts[0][1], 14), 0, 0, -90)
        for vals in pts: 
            print(vals)
            #self.rotate_gripper(vals[0], vals[1])
            result = self.go_to_location(vals, 0, 0, -90)
            time.sleep(0.5)
            #print(f"result {result}")
        for x in range (2):
            result = self.go_to_location((pts[x][0], pts[x][1], pts[x][2]), 0, 0,-90 )
            time.sleep(0.5)
        #result = self.go_to_location((0.53,20.23, 7), 0, 0, -90)
        #print(result)
        #print("at 0.53,20.23,7")
        #self.rotate_gripper(pts[-1][0], pts[-1][1])
        result = self.go_to_location((pts[2][0], pts[2][1], 12), 0, 0, -90)
        #self.grippers(True)

    def drawline(self, point_a, point_b):
        print("in drawline")
        # Calculate the distance between the two points
        distance = ((point_b[0] - point_a[0]) + (point_b[1] - point_a[1]))*0.5

        # Calculate the number of intermediate points
        num_intermediate_points = abs(int(distance)) + 1
        print(f"num pts: {num_intermediate_points}")
        # Calculate the step size for each coordinate
        step_x = (point_b[0] - point_a[0]) / num_intermediate_points
        step_y = (point_b[1] - point_a[1]) / num_intermediate_points
        print (f"step_x: {step_x}")
        print (f"step_y: {step_y}")
        # Simulate moving the brick along the straight line
        for i in range(num_intermediate_points + 1):
            intermediate_point_up = (point_a[0] + i * step_x, point_a[1] + i * step_y, 15)
            intermediate_point = (point_a[0] + i * step_x, point_a[1] + i * step_y, 5)
            if self.count_line == 0: 
                    self.go_to_location(intermediate_point_up, -90, -90, 0, movetime=500)
                    print("starting up")
            print(f"Moving to point {intermediate_point}")

            # move the brick to intermediate point

            #self.state.AK.setPitchRangeMoving(intermediate_point, -90, -90, 0, move_time=500) 
            self.go_to_location(intermediate_point, -90, -90, 0, movetime=500)#adjust function calling as implemented

            if self.count_line == num_intermediate_points -1 : 
                    self.go_to_location(intermediate_point_up, -90, -90, 0, movetime=500)
                    print("moving up again")

            self.count_line = self.count_line + 1

            time.sleep(1)
        self.count_line = 0 


        

if __name__ == '__main__':
    my_camera = Camera.Camera()
    motion = Motion()
    color = 'red'
    motion.run(color, my_camera)

    circle = Circle()
    circle.starting_position()
    #print("going to staring pos ")
    circle.run()
    #circle.drawline((0.53,20.23),( 0.53,15))
    time.sleep(1)
    circle.drawline((-1,31),(-1,29))
    time.sleep(1)
    circle.drawline((1,31),(1,29))
    time.sleep(1)
    circle.drawline((-2,28), (2,28))
    #circle.drawline((3,20.23),( 3,15))
    #circle.run()






    





        
        
        
             
        
        
              