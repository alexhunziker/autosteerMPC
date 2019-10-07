import time

import cv2
import matplotlib.pyplot as plt
import numpy as np

from .curve_calculator import CurveCalculator
from .edge_image_preprocessor import EdgeImagePreprocessor
from .image_warper import ImageWarper


class SecondOrderEdgeRecognizer(object):
    DEFAULT_DESTINATION_SIZE = (800, 600)

    def __init__(self, destination_size=DEFAULT_DESTINATION_SIZE, debug=False):
        self.destination_size = destination_size
        self.image_preprocessor: EdgeImagePreprocessor = EdgeImagePreprocessor()
        self.image_warper: ImageWarper = ImageWarper()
        self.curve_calculator: CurveCalculator = CurveCalculator(meters_per_pixel_x=2 / 800,
                                                                 meters_per_pixel_y=15 / 600, debug=debug, mode="R")
        self.img = None
        self.debug = debug

    def process(self, img):
        start_time = time.time()
        edges_image = self.image_preprocessor.process(img)
        edges_image = cv2.fastNlMeansDenoising(edges_image, searchWindowSize=9, templateWindowSize=5)
        source_roi = np.float32(
            [(0.3, 0.3), (0.48, 0.3), (0, 1), (1, 1)])  # TODO: This needs to be adjusted to final footage
        warped_image = self.image_warper.warp(edges_image, destination_size=self.destination_size,
                                              source_roi_proportion=source_roi)
        warped_image = cv2.Canny(warped_image, 0, 1)
        self.curve_calculator.recognize_curve(warped_image, minpix=40)
        self.img = img
        curverad = self.get_curve_radius()
        lane_curvature = curverad[1]
        print("INFO: Curvature (m):", lane_curvature, "vehicle offset (m):", curverad[2])
        print("INFO: Image processed in ", time.time() - start_time, "s")
        return lane_curvature, curverad[2]

    def get_curve_radius(self):
        return self.curve_calculator.fit_curve_worldspace(self.img, mode="R")

    def visualize_lane(self):
        lanes = self.curve_calculator.draw_lanes(self.img)
        curverad = self.get_curve_radius()
        lane_curve = curverad[1]

        font = cv2.FONT_HERSHEY_SIMPLEX
        fontColor = (0, 0, 0)
        fontSize = 0.5
        cv2.putText(lanes, 'Lane Curvature: {:.0f} m'.format(lane_curve), (570, 620), font, fontSize, fontColor, 2)
        cv2.putText(lanes, 'Vehicle offset: {:.4f} m'.format(curverad[2]), (570, 650), font, fontSize, fontColor, 2)
        return lanes


# Simplistic approach for testing
if __name__ == "__main__":
    img = cv2.imread('resources/campus_straight.jpg')
    secondOrderEdgeRecognizer = SecondOrderEdgeRecognizer(debug=False, destination_size=(800, 600))
    for i in range(10):
        print("Processing result:", secondOrderEdgeRecognizer.process(img))
    curverad = secondOrderEdgeRecognizer.get_curve_radius()
    result_img = secondOrderEdgeRecognizer.visualize_lane()
    plt.imshow(result_img, cmap="hsv")
    plt.show()
