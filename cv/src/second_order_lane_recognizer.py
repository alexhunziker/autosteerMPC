import time

import cv2
import matplotlib.pyplot as plt
import numpy as np

from image_preprocessor import ImagePreprocessor
from image_warper import ImageWarper
from lane_recognizer import LaneRecognizer


class SecondOrderLaneRecognizer(object):

    def __init__(self):
        self.image_preprocessor: ImagePreprocessor = ImagePreprocessor()
        self.image_warper: ImageWarper = ImageWarper()
        self.lane_recognizer: LaneRecognizer = LaneRecognizer()
        self.img = None

    def process(self, img):
        start_time = time.time()
        edges_image = self.image_preprocessor.process(img)
        warped_image = self.image_warper.warp(edges_image)
        self.lane_recognizer.sliding_window(warped_image)
        self.img = img
        print("Image processed in ", time.time() - start_time, "s")
        return self

    def get_curve_radius(self):
        return self.lane_recognizer.fit_curve_worldspace(self.img)

    def visualize_lane(self):
        lanes = self.lane_recognizer.draw_lanes(self.img)
        curverad = self.get_curve_radius()
        lane_curve = np.mean([curverad[0], curverad[1]])

        font = cv2.FONT_HERSHEY_SIMPLEX
        fontColor = (0, 0, 0)
        fontSize = 0.5
        cv2.putText(lanes, 'Lane Curvature: {:.0f} m'.format(lane_curve), (570, 620), font, fontSize, fontColor, 2)
        cv2.putText(lanes, 'Vehicle offset: {:.4f} m'.format(curverad[2]), (570, 650), font, fontSize, fontColor, 2)
        return lanes


# Simplistic approach for testing
if __name__ == "__main__":
    mode = "picture"

    if mode == "picture":
        img = cv2.imread('resources/curve_1.jpg')
        # img = cv2.imread('resources/road_with_fixes.jpg')
        # img = cv2.imread('resources/light_curve_2.jpg')
        secondOrderLaneRecognizer = SecondOrderLaneRecognizer().process(img)
        curverad = secondOrderLaneRecognizer.get_curve_radius()
        print(curverad)
        result_img = secondOrderLaneRecognizer.visualize_lane()
        plt.imshow(result_img, cmap="hsv")
        plt.show()
