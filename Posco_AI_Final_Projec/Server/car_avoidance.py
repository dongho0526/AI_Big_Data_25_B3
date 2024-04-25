import cv2
import numpy as np

def forward(direction):
    if direction == 'left':
        print('left')
    elif direction == 'right':
        print('right')
    elif direction == 'special':
        print('special')
        


def get_lane_center(lane_number, image_width):
    """ 각 차선의 중앙 위치를 반환합니다. """
    lane_width = image_width / 3
    lane_center = int(lane_width * (lane_number - 0.5))
    return lane_center

def get_bbox_area(bbox):
    """ 주어진 바운딩 박스의 면적을 계산합니다. """
    xmin, ymin, xmax, ymax = bbox
    return (xmax - xmin) * (ymax - ymin)

def avoid_vehicles(frame, cls, bboxes, current_lane, x_value_at_max_width, bboxes_xywh):
    image_width = frame.shape[1]
    lane_info = {'left': 0, 'right': 0, 'current': 0}
    
    for idx, bbox in enumerate(bboxes):
        xmin, ymin, xmax, ymax = bbox
        bbox_center = (xmin + xmax)//2  # cx 
        bbox_area = bboxes_xywh[idx][2] * bboxes_xywh[idx][3]
        lane_center = get_lane_center(current_lane, image_width)
        
        if bbox_center < x_value_at_max_width - 30:
                lane_info['left'] = max(lane_info['left'], bbox_area)
        elif bbox_center >  x_value_at_max_width + 30:
                lane_info['right'] = max(lane_info['right'], bbox_area)
        else:
                lane_info['current'] = max(lane_info['current'], bbox_area)
    print('lane_info: ', lane_info)
    print('lane_info: ', lane_info)
    print('lane_info: ', lane_info)
    print('lane_info: ', lane_info)
    
    # current_lane = 2
    # 가장 가까운 차량 거리 측정  current_lane > 1 and 
    if current_lane ==1:
        # print('left')
        # print('left')
        # print('left')
        # print('left')
        return current_lane+1, 5.0
    elif (lane_info['left'] < lane_info['right']):
        # cv2.putText(frame, 'left!', (300, 300), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
        # print('left')
        # print('left')
        # print('left')
        # print('left')
        return current_lane-1, 1.0
    elif (lane_info['left'] > lane_info['right']):
        # cv2.putText(frame, 'right!', (300, 300), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
        # print('right')
        # print('right')
        # print('right')
        # print('right')
        return current_lane+1, 3.0
    # elif current_lane == 3 and lane_info['current'] > 5000: # size adjustment
        # print('left')
        # print('left')
        # print('left')
        # print('left')
        # return current_lane-1, 1.0
    # print('stay')
    # print('stay')
    # print('stay')
    # print('stay')
    return current_lane, 2.0

# 예시 사용
# frame = cv2.imread('path_to_image.jpg')  # 이미지 로드
# bboxes = [(100, 150, 200, 300, 2, 0.9), (300, 100, 400, 250, 2, 0.95), (500, 120, 600, 270, 2, 0.85)]
# decision = avoid_vehicles(frame, bboxes)
# print(decision)

                