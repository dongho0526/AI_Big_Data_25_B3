import cv2
import math
import numpy as np

class ControlMoving:
    def __init__(self, avoid_point_center, avoid_point_right, avoid_point_left):
        self.avoid_point_center = avoid_point_center
        self.avoid_point_right = avoid_point_right
        self.avoid_point_left = avoid_point_left
        self.start_point = (320, 480)
        # self.end_tp_x = None
        # self.start_tp_y = None
        # self.end_tp_y = None
        # self.pos = None         # vehicle position from center
        self.target_point_x = None
        self.target_point_y = None
        self.label_list = None
        self.left_speed = 0.5
        self.right_speed = 0.5
    
    # def get_object(self, frame):
    #     frame.box
    # def get_angle(self):    # positive value -> turn right -> five force to left
    #     # print("Self=", self.start_point[0])
    #     try:
    #         x = self.target_point_x - self.start_point[0]
    #         y = self.target_point_y - self.start_point[1]
    #         # * (180 / math.pi)
    #         # print(self.target_point_x, self.target_point_y)
    #         # print(self.target_point_x, self.target_point_y)
    #         # print(self.target_point_x, self.target_point_y)
    #         # print(self.target_point_x, self.target_point_y)
    #         # print(self.target_point_x, self.target_point_y)
    #         # print(self.target_point_x, self.target_point_y)
    #         # print(self.target_point_x, self.target_point_y)
    #         return 90.0 - np.arctan(y/x)  # to radian
    #     except ZeroDivisionError:
    #         print("0으로 나눌 수 없습니다.")
    #     except TypeError:
    #         print("None value")
            
        
        # x = self.start_point[0] - self.target_point_x
        # y = self.start_point[1] - self.target_point_y
         

    # def go_lane(self):   # return handle angle
    #     # balance = 1
    #     # balance = self.pos * c
    #     # if self.pos == 0:
    #     #     balance = 1
    #     # return balance
    #     # if self.pos < 0: # (0.97, c*1)
    #     #     pass
    #     # elif self.pos > 0: # (0.97*c, 1)
    #         # pass
    #     # print(self.start_point[0])
    #     angle = self.get_angle()
        # print(angle)
        # print(angle)
        # print(angle)
        # print(angle)
        # print(angle)
        # print(angle)
        # print(angle)
        # print(angle)

        
        

        
        
    
    def move(self, frame, target_point_x, target_point_y, left_speed, right_speed, largest_contour_position):
        self.label_list = frame[0].names
        self.target_point_x = target_point_x
        self.target_point_y = target_point_y
        self.left_speed = left_speed
        self.right_speed = right_speed
        self.largest_contour_position = largest_contour_position
        # self.go_lane()
        for box in frame[0].boxes:
            # x1, y1, x2, y2 = box.xyxy[0]    # top left point, bottom right point
            cx, cy, w, h = box.xywh[0]      # center x, center y, w, h
            label = self.label_list[int(box.cls.item())]
            # print("self.pos =",self.pos)
            # print("self.pos =",self.pos)

            if label == 'car':  # avoid 
                if (self.avoid_point_center[0] < int(cx)+int(w//2)) and (self.avoid_point_center[0] > int(cx)-int(w//2)) and \
                (self.avoid_point_center[1] < int(cy)+int(h//2)) and (self.avoid_point_center[1] > int(cy)-int(h//2)):
                    # cv2.putText(frame[0].plot(), "Run avoid!!!", org=(400, 240), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 255, 255), thickness=2)
                    print("RUN AVOID!!!!!!!!!")
                    print("RUN AVOID!!!!!!!!!")
                    print("RUN AVOID!!!!!!!!!")
                    print("RUN AVOID!!!!!!!!!")
                    print("RUN AVOID!!!!!!!!!")
                    print("RUN AVOID!!!!!!!!!")
                    print("RUN AVOID!!!!!!!!!")
                    print("RUN AVOID!!!!!!!!!")
            if label == 'cat' or label == 'person' or largest_contour_position == 'red':     # stop (+ light signal)
                if (self.avoid_point_center[0] < int(cx)+int(w//2)) and (self.avoid_point_center[0] > int(cx)-int(w//2)) and \
                (self.avoid_point_center[1] < int(cy)+int(h//2)) and (self.avoid_point_center[1] > int(cy)-int(h//2)):
                    print("STOP!!!!!!!!!")
                    print("STOP!!!!!!!!!")
                    print("STOP!!!!!!!!!")
                    print("STOP!!!!!!!!!")
                    print("STOP!!!!!!!!!")
                    print("STOP!!!!!!!!!")
                    print("STOP!!!!!!!!!")
                    self.left_speed = 0
                    self.right_speed = 0
            
        return self.left_speed, self.right_speed
            
        # return frame
        # get_object(frame)
        
    
    