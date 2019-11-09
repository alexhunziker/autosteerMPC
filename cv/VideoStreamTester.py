import time
from statistics import mean

import cv2
from .second_order_lane_recognizer import SecondOrderLaneRecognizer

times_retrieved_img = []
global_start = time.time()
frames = 0
times_preprocessed = []
times_calculated_lanes = []
intermediate = time.time()

lane_recognizer = SecondOrderLaneRecognizer()
cap = cv2.VideoCapture("resources/curve_test.mp4")
while cap.isOpened():
    start = time.time()
    _, frame = cap.read()
    times_retrieved_img.append(int((time.time() - start) * 1000))
    try:
        lane_recognizer.process(frame)
        result = lane_recognizer.visualize_lane()
    except:
        break
    cv2.imshow('result', result)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    print(int((time.time() - start) * 1000), "ms. Intermediate ", int((time.time() - intermediate) * 1000), "ms")
    intermediate = time.time()
    frames += 1

print("It took: ", int((time.time() - global_start) * 1000), "ms. Frames: ", frames)

print("Retrieved image after:", mean(times_retrieved_img), "ms")

cap.release()
cv2.destroyAllWindows()
