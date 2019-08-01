import time
from statistics import mean

import cv2
import numpy as np


def canny(image):
    grey = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(grey, (5, 5), 0)  # Reduce noise
    # low and high threshold, rejected if below low, accepted only if close to hard edge if between high and low
    return cv2.Canny(blur, 50, 150)


def region_of_interest(image):
    height: int = image.shape[0]
    # Only this triangle contains the road... is this a smart concept (what about curves) ???
    polygons = np.array([
        [(200, height), (1100, height), (550, 250)]
    ])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


def average_slope_intercept(image, lines):
    left_fit = []
    right_fit = []
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        intercept = parameters[1]
        if slope < 0:
            left_fit.append((slope, intercept))
        else:
            right_fit.append((slope, intercept))
    left_fit_average = np.average(left_fit, axis=0)
    right_fit_average = np.average(right_fit, axis=0)
    try:
        left_line = make_coordinates(image, left_fit_average)
    except:
        left_line = np.array([0, 0, 0, 0])
    right_line = make_coordinates(image, right_fit_average)
    return np.array([left_line, right_line])


def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = int(y1 * (3 / 5))
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])


def display_lines(image, lines, rgb):
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            cv2.line(line_image, (x1, y1), (x2, y2), rgb, 10)
    return line_image


# image = cv2.imread('resources/test_image.jpg')
# lane_image = np.copy(image)
# canny_image = canny(lane_image)
# cropped_image = region_of_interest(canny_image)
# lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)
# average_lines = average_slope_intercept(lane_image, lines)
# line_image = display_lines(lane_image, average_lines)
# combo_image = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1)
# cv2.imshow('result', combo_image)
# cv2.waitKey(0)

global_start = time.time()
intermediate = time.time()
frames = 0
times_retrieved_img = []
times_preprocessed = []
times_calculated_lanes = []
cap = cv2.VideoCapture("resources/test2.mp4")
while cap.isOpened():
    start = time.time()
    _, frame = cap.read()
    times_retrieved_img.append(int((time.time() - start) * 1000))
    try:
        canny_image = canny(frame)
    except:
        break
    cropped_image = region_of_interest(canny_image)
    times_preprocessed.append(int((time.time() - start) * 1000))
    lines = cv2.HoughLinesP(cropped_image, 2, np.pi / 180, 100, np.array([]), minLineLength=30, maxLineGap=5)
    average_lines = average_slope_intercept(frame, lines)
    times_calculated_lanes.append(int((time.time() - start) * 1000))
    line_image = np.zeros_like(frame)
    line_image = display_lines(line_image, lines, (0, 225, 0))
    line_image = display_lines(line_image, average_lines, (225, 0, 0))
    combo_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    # cv2.imshow('result', combo_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    print(int((time.time() - start) * 1000), "ms. Intermediate ", int((time.time() - intermediate) * 1000), "ms")
    intermediate = time.time()
    frames += 1

print("It took: ", int((time.time() - global_start) * 1000), "ms. Frames: ", frames)

print("Retrieved image after:", mean(times_retrieved_img), "ms")
print("Preprocessed after:", mean(times_preprocessed), "ms")
print("Calculated lanes after:", mean(times_calculated_lanes), "ms")

cap.release()
cv2.destroyAllWindows()
