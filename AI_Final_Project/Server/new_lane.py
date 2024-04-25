import numpy as np
import cv2

lower_yellow = np.array([10, 80, 80])
upper_yellow = np.array([40, 255, 255])
lower_threshold = 200

def new_lane(frame, past_center_of_lane):
    height = frame.shape[0]
    crop_img = frame[height-30:height-10, :]
    white_pixels_lower_threshold = np.where(np.all(crop_img >= [lower_threshold, lower_threshold, lower_threshold], axis=-1))
    white_pixel_coords_lower_threshold = list(zip(white_pixels_lower_threshold[1], white_pixels_lower_threshold[0] + height - 30))
    
    yellow_pixels_threshold = np.all(crop_img >= lower_yellow, axis=-1)
    yellow_pixels_threshold = np.where(np.all(crop_img <= upper_yellow, axis=-1))
    yellow_pixels_coords_threshold = list(zip(yellow_pixels_threshold[1], yellow_pixels_threshold[0] + height - 30))

    x_left_max = 0
    x_right_min = 320

    # print(white_pixel_coords_lower_threshold)
    for coord in white_pixel_coords_lower_threshold:
        # 좌표에 빨간 점을 찍습니다. 여기서는 점의 크기를 2로 설정했습니다.
        cv2.circle(frame, (coord[0], coord[1]), 2, (0, 0, 255), -1)
        if 0 <= coord[0] < 160:
            x_left_max = max(x_left_max, coord[0])
        elif 160 <= coord[0] < 320:
            x_right_min = min(x_right_min, coord[0])

    for coord in yellow_pixels_coords_threshold:
        cv2.circle(frame, (coord[0], coord[1]), 2, (0, 0, 255), -1)
        x_left_max = max(x_left_max, coord[0])
# len(white_pixel_coords_lower_threshold) > 2 :a
    wid = int(x_right_min - x_left_max)
    mid = (x_left_max+x_right_min)//2
    # print("wid =", wid)
    # print("wid =", wid)
    # print("wid =", wid)
    # print("wid =", wid)
    # print("wid =", wid)
    # print("wid =", wid)
    # print("wid =", wid)
    # print("wid =", wid)
    # print("wid =", wid)
    # print("wid =", wid)
    # print("wid =", wid)
    # print("wid =", wid)
    # mid > 100 
    if wid > 130 and wid < 150:    
        return mid
    return past_center_of_lane
        