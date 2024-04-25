import cv2
import time
import numpy as np
from Thresholding import Thresholding
from PerspectiveTransformation import PerspectiveTransformation
from LaneLines import LaneLines
from new_lane import new_lane
from ControlMoving import ControlMoving
from ultralytics import YOLO  # 예시로 포함된 라이브러리
import socket
import pickle
import struct
from car_avoidance import avoid_vehicles


# 0: person, 2: car, 5: bus, 7: truck, 9: traffic light 15: cat

###############################################################
wanted_class_dict = {0: 'person', 3 : 'car', 2: 'car', 5: 'bus', 7: 'truck', 9: 'traffic light', 15: 'cat'}

class_colors = {
    0: (255, 255, 255),    # person: blue
    2: (0, 255, 0),    # car: green
    3: (0, 255, 0),
    5: (0, 0, 255),    # bus: red
    7: (255, 0, 255),  # truck: cyan
    9: (255, 255, 0),  # traffic light: magenta
    15: (0, 255, 255)  # cat: yellow
}
#############################################################33

avoid_point_center = (320, 410)
avoid_point_right = (160, 240)
avoid_point_left = (480, 240)

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # 비례 상수
        self.ki = ki  # 적분 상수
        self.kd = kd  # 미분 상수
        self.previous_error = 0
        self.integral = 0

    def update(self, error, dt):
        """ PID 계산을 업데이트하고 제어 값을 반환합니다.
        :param error: 현재 오차 (선의 중심과 차량의 중심 사이)
        :param dt: 이전 업데이트 이후의 시간 간격
        :return: 제어 출력
        """
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output


def control_vehicle(center_of_lane, center_of_vehicle, pid_controller, dt, state):
    left_motor_speed, right_motor_speed = 0, 0
    if state == 'stop':
        return left_motor_speed, right_motor_speed
    
    if state == 'emergency':
        left_motor_speed, right_motor_speed = 0.6, 0.6
        return left_motor_speed, right_motor_speed
    
    # 오차 계산
    error = center_of_lane - center_of_vehicle
    
    # PID 제어기 업데이트
    control = pid_controller.update(error, dt)
    # 모터 속도 계산
    base_speed = 0.8
    # 기본 속도
    left_motor_speed = base_speed + control*0.005
    right_motor_speed = base_speed - control*0.005
    
    # 속도 제한
    left_motor_speed = max(min(left_motor_speed, 1.0), 0.0)
    right_motor_speed = max(min(right_motor_speed, 1.0), 0.0)
    
    return left_motor_speed, right_motor_speed


################calculate_bounding_box_size##############

def calculate_bounding_box_size(size_list):
    start_x, start_y, end_x, end_y = size_list[0], size_list[1], size_list[2], size_list[3]
    width = end_x - start_x

    height = end_y - start_y

    return width * height    

#######################################################


def get_contour_bounding_rect_area(contour):
    x, y, w, h = cv2.boundingRect(contour)
    return w * h

def extract_white_colors(image):

    # 흰색 범위 설정
    lower_white = np.array([220, 220, 220])
    upper_white = np.array([255, 255, 255])
    # 흰색 영역에 대한 마스크 생성
    white_mask = cv2.inRange(image, lower_white, upper_white)
    # 마스크 적용: 흰색 영역만 보존하고 나머지는 검은색으로 처리
    result = cv2.bitwise_and(image, image, mask=white_mask)
    return result

def is_circle_like(contour, circularity_threshold=0.75):
    # Calculate area of the contour
    area = cv2.contourArea(contour)
    # Calculate the perimeter (arc length) of the contour
    perimeter = cv2.arcLength(contour, True)
    # Calculate circularity
    circularity = 4 * np.pi * (area / (perimeter ** 2)) if perimeter > 0 else 0
    # Return True if circularity is greater than the threshold
    return circularity > circularity_threshold

class FindLaneLines:
    def __init__(self):
        self.thresholding = Thresholding()
        self.transform = PerspectiveTransformation()
        self.lanelines = LaneLines()
        # self.control = ControlMoving(avoid_point_center, avoid_point_right, avoid_point_left)
        self.target_point_x = None
        self.target_point_y = None
        self.past_center_of_lane = 160
        self.state = None
        
    def forward(self, img):
        out_img = np.copy(img)
        img = self.transform.forward(img)
        img = self.thresholding.forward(img)
        img, self.target_point_x, self.target_point_y,self.diff = self.lanelines.forward(img)
        img = self.transform.backward(img)
        out_img = cv2.addWeighted(out_img, 1, img, 0.6, 0)
        # out_img = self.lanelines.plot(out_img)
        return out_img

    def process_webcam(self):
        #cap = cv2.VideoCapture(0)  # 웹캠 초기화
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        fps_counter = 0
        fps_start_time = time.time()
        current_lane = 2
        decision = 4.0
        
        
        while True:
            traffic_color = 'None'
            client_socket, addr = server_socket.accept()
            print(f"{addr}에서 연결되었습니다.")

            data = b""
            payload_size = struct.calcsize("Q")  # unsigned long long의 크기로 변경
            
            while True:
            # 프레임 크기 정보 수신
                while len(data) < payload_size:
                    packet = client_socket.recv(4*1024)  # 4K
                    if not packet: break
                    data += packet
                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack("Q", packed_msg_size)[0]  # 'L'을 'Q'로 변경

                # 프레임 데이터 수신
                while len(data) < msg_size:
                    data += client_socket.recv(4*1024)
                frame_data = data[:msg_size]
                data = data[msg_size:]
                # 데이터를 프레임으로 디코딩
                frame = pickle.loads(frame_data)
                
            
                
                frame = model(frame, iou=0.4)  # YOLO 모델 사용 예시
                orig_img = frame[0].orig_img
                # processed_frame = self.forward(frame[0].plot())  # YOLO 모델과의 호환 가정
                orig_img = self.forward(orig_img)
                #self.control.move(frame, self.target_point_x, self.target_point_y)       # 가정된 예제 메소드
                cls = frame[0].boxes.cls.cpu().tolist()
                xyxy = frame[0].boxes.xyxy.cpu().tolist()
                xywh = frame[0].boxes.xywh.cpu().tolist()

                ################### New Bounding Box #######################
                if cls != []:
                    for idx, cls_num in enumerate(cls):
                        if cls_num in wanted_class_dict.keys():
                            bbox = xyxy[idx]
                            bbox = list(map(int, bbox))
                            color = class_colors.get(cls_num, (0, 0, 0))
                            cv2.rectangle(orig_img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
                            cv2.putText(orig_img, wanted_class_dict[cls_num], (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_COMPLEX, 0.4, color, 1)
                ##############################################################

                if 9 in cls:
                    # img = frame[0].orig_img
                    traffic_bbox = xyxy[cls.index(9)]
                    traffic_bbox = list(map(int, traffic_bbox))
                    img_cropped = orig_img[traffic_bbox[1]:traffic_bbox[3], traffic_bbox[0]:traffic_bbox[2]]
                    white = extract_white_colors(img_cropped)
                    gray = cv2.cvtColor(white, cv2.COLOR_BGR2GRAY)
                    _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
                    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    circle_like_contours = [cnt for cnt in contours if is_circle_like(cnt)]
                    if circle_like_contours != [] :        
                        sorted_circle_like_contours = sorted(circle_like_contours, key=get_contour_bounding_rect_area, reverse=True)
                        largest_contour = sorted_circle_like_contours[0]
                        largest_contour = sorted_circle_like_contours[0]
                        # Get the bounding rect for the largest contour
                        x, y, w, h = cv2.boundingRect(largest_contour)
                        # Determine if the largest contour is in the upper or lower half of the image
                        image_center_y = gray.shape[0] / 2
                        traffic_color = "red" if y + h / 2 < image_center_y else "green"
                        # print(largest_contour_position)
                        if traffic_color == 'red':
                            
                            cv2.putText(orig_img, traffic_color, (260, 20), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

                        elif traffic_color == 'green':
                            cv2.putText(orig_img, traffic_color, (240, 20), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                
                    traffic_size = ((gray.shape[0]/2) * (gray.shape[1]/2)) # traffic_ligth size calculation

                ################# Traffic Ligth Red & Stop #################
                    print('traffic_size: ', traffic_size)
                    print('traffic_size: ', traffic_size)
                    print('traffic_size: ', traffic_size)
                    if traffic_size > 1250: # size 조정 필요
                        if traffic_color == 'red':
                            cv2.putText(orig_img, 'Stop!', (120, 20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
                            self.state = 'emergency'
                # print(traffic_color)
                ############################################################

                
                ###################### Car Avoidance ##########################
                # print(cls)
                if 2 in cls and xyxy != []:
                    filtered_cls = [x for x in cls if x == 2]
                    filtered_xyxy = [xyxy[i] for i in range(len(filtered_cls)) if cls[i] == 2]
                    filtered_xywh = [xywh[i] for i in range(len(filtered_cls)) if cls[i] == 2]
                    if filtered_xyxy != []:
                        max_y = 0
                        mid_box = None
                        for i, (_, y, _, _) in enumerate(filtered_xywh):
                            if y > max_y:
                                max_y = y
                                mid_box = i
        
                        # Retrieve the 'x' value at the index of the maximum width
                        
                        # print(filtered_xywh)
                        x_value_at_max_y = filtered_xywh[mid_box][0] # center_point  cx
                        # print(x_value_at_max_y)
                        # print(x_value_at_max_y)
                        value_adjust = 50
                        if x_value_at_max_y < orig_img.shape[1]/2 + value_adjust and x_value_at_max_y > orig_img.shape[1]/2 - value_adjust and max_y > 2*(orig_img.shape[0]/3):
                            current_lane, decision = avoid_vehicles(orig_img, filtered_cls, filtered_xyxy, current_lane, x_value_at_max_y, filtered_xywh)
                # elif 2 in cls and xyxy == []:
                #     decision = 4.0
                ###############################################################
                
                if 3 in cls:
                    motorcycle_bbox = xyxy[cls.index(3)]
                    motorcycle_mid = (motorcycle_bbox[0] + motorcycle_bbox[2])/2
                    
                    motorcycle_bbox = list(map(int, motorcycle_bbox))
                    motorcycle_size = calculate_bounding_box_size(motorcycle_bbox)
                    print('motorcycle_size: ', motorcycle_size)
                    print('motorcycle_size: ', motorcycle_size)
                    print('motorcycle_size: ', motorcycle_size)
                    if motorcycle_mid > 250:
                        self.state = None
                    else:
                        if motorcycle_size > 6000: # size 조정 필요
                            cv2.putText(orig_img, 'Stop!', (120, 20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
                            self.state = 'stop'
                

                ################### Person Detect & Stop ###################
                if 0 in cls:
                    person_bbox = xyxy[cls.index(0)]
                    person_bbox = list(map(int, person_bbox))
                    person_size = calculate_bounding_box_size(person_bbox)
                    print('person_size: ', person_size)
                    print('person_size: ', person_size)
                    print('person_size: ', person_size)
                    if person_size > 1500: # size 조정 필요
                        cv2.putText(orig_img, 'Stop!', (120, 20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
                        self.state = 'stop'

                ############################################################

                ##################### Cat Detect & Stop ####################
                if 15 in cls:
                    cat_bbox = xyxy[cls.index(15)]
                    cat_bbox = list(map(int, cat_bbox))
                    cat_size = calculate_bounding_box_size(cat_bbox)
                    print('cat_size: ', cat_size)
                    print('cat_size: ', cat_size)
                    print('cat_size: ', cat_size)
                    if cat_size > 1200: # size 조정 필요
                        cv2.putText(orig_img, 'Stop!', (120, 20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
                        self.state = 'stop'

                #############################################################

                
                center_of_lane = new_lane(orig_img, self.past_center_of_lane)
                center_of_vehicle = orig_img.shape[1]/2
                print('--------midpoint----------')
                print(center_of_vehicle, center_of_lane) # midpoint on the image, midpoint on the lane
                print('--------------------------')
                pid = PIDController(kp=0.01, ki = 0.01, kd=0.05)
                
                # 처리된 프레임에 시각적 요소 추가
                # cv2.line(orig_img, (320, 480), avoid_point_center, (255, 0, 0), thickness=5)
                # cv2.circle(orig_img, avoid_point_center, 8, (0, 0, 255), thickness=5)
                # cv2.line(orig_img, (320, 480), avoid_point_right, (255, 0, 0), thickness=5)
                # cv2.circle(orig_img, avoid_point_right, 8, (0, 0, 255), thickness=5)
                # cv2.line(orig_img, (320, 480), avoid_point_left, (255, 0, 0), thickness=5)
                # cv2.circle(orig_img, avoid_point_left, 8, (0, 0, 255), thickness=5)
                # cv2.circle(orig_img, (272, 410), 20, (255, 255, 0), thickness=5)

                
                fps_counter += 1
            
                fps = fps_counter / (time.time() - fps_start_time)
                left_speed, right_speed = control_vehicle(center_of_lane, center_of_vehicle, pid, fps, self.state)
                self.state = None
                self.past_center_of_lane = center_of_lane
                
                print('----------speed---------')
                print(left_speed, right_speed)
                print(left_speed, right_speed)
                fps_text = f"FPS: {fps:.2f}"
                cv2.putText(orig_img, fps_text, (20, 20), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 1,cv2.LINE_AA)
                fps_counter = 0
                fps_start_time = time.time()
                
                cv2.line(orig_img, (center_of_lane, 239), (center_of_lane, 200), (0, 0, 255), 2, cv2.LINE_AA)
                cv2.line(orig_img, (160, 239), (160, 200), (255, 0, 0), 2, cv2.LINE_AA)


                cv2.imshow('Lane Detection', orig_img)
                if left_speed is None or right_speed is None:
                    print("speed value is None")
                    left_speed = 10.0
                    right_speed = 10.0
            
                packed_data = struct.pack('fff', left_speed, right_speed, decision)
                
                client_socket.sendall(packed_data)
                decision = 4.0
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q'):
                    break
                    
            # cap.release()
            # cv2.destroyAllWindows()

def main():
    findLaneLines = FindLaneLines()
    findLaneLines.process_webcam()
    
if __name__ == "__main__":
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host_ip = ''  # 모든 주소에서 접근 가능하게 설정 (우분투 주소로 변경)
    port = 9927  # 포트 번호
    server_socket.bind((host_ip, port))
    server_socket.listen(5)
    
    print("서버 대기 중...")
    model = YOLO('yolov8n.pt')  # YOLO 모델 초기화
    main()