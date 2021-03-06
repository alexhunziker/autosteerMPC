import time

import cv2
import matplotlib.pyplot as plt
import numpy as np

from .curve_calculator import CurveCalculator
from .image_preprocessor import ImagePreprocessor
from .image_warper import ImageWarper


class SecondOrderLaneRecognizer(object):
    DEFAULT_DESTINATION_SIZE = (1280, 720)

    def __init__(self, destination_size=DEFAULT_DESTINATION_SIZE, debug=False):
        self.destination_size = destination_size
        self.image_preprocessor: ImagePreprocessor = ImagePreprocessor()
        self.image_warper: ImageWarper = ImageWarper()
        self.curve_calculator: CurveCalculator = CurveCalculator(debug=debug)
        self.img = None
        self.debug = debug

    def process(self, img):
        start_time = time.time()
        edges_image = self.image_preprocessor.process(img)
        warped_image = self.image_warper.warp(
            edges_image, destination_size=self.destination_size)
        self.curve_calculator.sliding_window(warped_image, left_lane=True)
        self.curve_calculator.sliding_window(warped_image, left_lane=False)
        self.img = img
        curverad = self.get_curve_radius()
        lane_curvature = np.mean([curverad[0], curverad[1]])
        print("INFO: Curvature (m):", lane_curvature,
              "vehicle offset (m):", curverad[2])
        print("curverad", curverad)
        print("INFO: Image processed in ", time.time() - start_time, "s")
        return lane_curvature, curverad[2]

    def get_curve_radius(self):
        return self.curve_calculator.fit_curve_worldspace(self.img)

    def visualize_lane(self):
        lanes = self.curve_calculator.draw_lanes(self.img, self.image_preprocessor.DEFAULT_SOURCE_ROI)
        curverad = self.get_curve_radius()
        lane_curve = np.mean([curverad[0], curverad[1]])

        font = cv2.FONT_HERSHEY_SIMPLEX
        fontColor = (0, 0, 0)
        fontSize = 1.3
        cv2.putText(lanes, 'Lane Curvature: {:.0f} m'.format(
            lane_curve), (480, 590), font, fontSize, fontColor, 3)
        cv2.putText(lanes, 'Vehicle offset: {:.4f} m'.format(
            curverad[2]), (480, 650), font, fontSize, fontColor, 3)
        return lanes


# Simplistic approach for testing
if __name__ == "__main__":
    mode = "lane"

    secondOrderLaneRecognizer = None
    if mode == "lane":
        img = cv2.imread('resources/lane_curve_1.jpg')
        secondOrderLaneRecognizer = SecondOrderLaneRecognizer(
            debug=True)
        secondOrderLaneRecognizer.process(img)
        # img = cv2.imread('resources/lane_edgecase_1.jpg')
        # img = cv2.imread('resources/lane_curve_2.jpg')
    if mode == "edge":
        img = cv2.imread('resources/straight_5.jpg')
        secondOrderLaneRecognizer = SecondOrderLaneRecognizer(
            debug=True, destination_size=(800, 600))
        secondOrderLaneRecognizer.process(img)
    curverad = secondOrderLaneRecognizer.get_curve_radius()
    print(curverad)
    result_img = secondOrderLaneRecognizer.visualize_lane()
    plt.imshow(result_img, cmap="hsv")
    plt.show()
