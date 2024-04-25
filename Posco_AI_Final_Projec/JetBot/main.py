import cv2
import socket
import struct
import pickle
from jetbot import Robot, Camera
import time
import cv2
import random
import threading



    
    


# A¡ì¢¬¨­¢Òo ¨ù©øA¢´
robot = Robot()


def right():
    print("right")
    
    
    start_time = time.time()
    while time.time() - start_time < 1:
       robot.set_motors(0.8, 0.65)
    
    start_time = time.time()
    while time.time() - start_time < 0.75:
       robot.set_motors(0.8, 0.8)
    
    start_time = time.time()
    while time.time() - start_time < 1.2:
       robot.set_motors(0.60, 0.82)
#     start_time = time.time()
#     while time.time() - start_time < 0.75:
#        robot.set_motors(0.8, 0.8)


def left():
    print("left")
    start_time = time.time()
    while time.time() - start_time < 1.7:
       robot.set_motors(0.65, 0.82)
    
    start_time = time.time()
    while time.time() - start_time < 1.3:
       robot.set_motors(0.8, 0.8)
    
    start_time = time.time()
    while time.time() - start_time < 1:
       robot.set_motors(0.75, 0.65)
    start_time = time.time()

    
#     while time.time() - start_time < 0.75:
#        robot.set_motors(0.8, 0.8)
    



#cap = cv2.VideoCapture(gstreamer_pipeline(framerate=60), cv2.CAP_GSTREAMER)
#cv2.VideoCapture('nvarguscamerasrc ! video/x-raw(memory:NVMM), width=320, height=240, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink', cv2.CAP_GSTREAMER)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
#cap = cv2.VideoCapture(create_gstreamer_pipeline(), cv2.CAP_GSTREAMER)
# ¨ùOAI ¨ù©øA¢´
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_ip = '141.223.140.40'  # ¢¯i¨¬¨¢Ao ¨ù¡©©öoAC IP AO¨ùO
port = 9920         # ¨ù¡©©öo¢¯I ¥ì¢¯AICN ¨¡¡À¨¡¢ç
SIZE = 12

client_socket.connect((host_ip, port))  # ¨ù¡©©öo¢¯¢® ¢¯¡þ¡Æa

def video_stream():
    while True:
        ret, frame = cap.read()
        if ret:
            data = pickle.dumps(frame)
            message_size = struct.pack("L", len(data)) 
            client_socket.sendall(message_size + data)
        if cv2.waitKey(10) == 13:
            break
    cap.release()
    client_socket.close()
    
def control_robot():
    while True:
        msg = client_socket.recv(12)
        left_speed, right_speed, decision = struct.unpack("fff", msg)
         print(left_speed, right_speed, decision )
         print(left_speed, right_speed, decision )
         print(left_speed, right_speed, decision )
         if decision == 4:
             robot.set_motors(left_speed, right_speed)
             print("none")
         elif decision == 3:
             right()
         elif decision == 1:
             left()
         elif decision == 5:
             left()
             robot.set_motors(left_speed, right_speed)
             time.sleep(1)
             right()
           
         else:
             robot.set_motors(left_speed, right_speed)
             print("stay")
        if cv2.waitKey(10) == 13:
            break
        robot.set_motors(left_speed, right_speed)


thread1 = threading.Thread(target=video_stream)
thread2 = threading.Thread(target=control_robot)

thread1.start()
thread2.start()

thread1.join()
thread2.join()


