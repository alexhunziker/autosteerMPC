import time

import cv2
import matplotlib.pyplot as plt
import numpy as np

from .curve_calculator import CurveCalculator
from .edge_image_preprocessor import EdgeImagePreprocessor
from .image_warper import ImageWarper


class SecondOrderEdgeRecognizer(object):
    DEFAULT_DESTINATION_SIZE = (800, 600)
    SOURCE_ROI = np.float32(
        [(0.25, 0.4), (0.75, 0.4), (0, 1), (1,
                                            1)])  # Empirically gathered, should be narrower on top, however then view angle is to narrow to view outward curves

    def __init__(self, destination_size=DEFAULT_DESTINATION_SIZE, debug=False):
        self.destination_size = destination_size
        self.image_preprocessor: EdgeImagePreprocessor = EdgeImagePreprocessor()
        self.image_warper: ImageWarper = ImageWarper()
        self.curve_calculator: CurveCalculator = CurveCalculator(meters_per_pixel_x=1.5 / 800,
                                                                 meters_per_pixel_y=4 / 400, debug=debug, mode="R")
        self.img = None
        self.debug = debug

    def process(self, img):
        start_time = time.time()
        edges_image = self.image_preprocessor.process(img)
        edges_image = cv2.fastNlMeansDenoising(edges_image, searchWindowSize=9, templateWindowSize=5)
        plt.imshow(edges_image)
        plt.show()
        warped_image = self.image_warper.warp(edges_image, destination_size=self.destination_size,
                                              source_roi_proportion=SecondOrderEdgeRecognizer.SOURCE_ROI)
        plt.imshow(warped_image)
        plt.show()
        warped_image = cv2.Canny(warped_image, 0, 1)
        self.curve_calculator.recognize_curve(warped_image, minpix=40, window_margin=60)
        self.img = img
        curverad = self.get_curve_radius()
        lane_curvature = curverad[1]
        if self.debug:
            print("INFO: Curvature (m):", lane_curvature, "vehicle offset (m):", curverad[2])
            print("INFO: Image processed in ", time.time() - start_time, "s")
        return lane_curvature, curverad[2]

    def get_curve_radius(self):
        return self.curve_calculator.fit_curve_worldspace(self.img, mode="R")

    def visualize_lane(self):
        lanes = self.curve_calculator.draw_lanes(self.img, SecondOrderEdgeRecognizer.SOURCE_ROI)
        curverad = self.get_curve_radius()
        lane_curve = curverad[1]
        offset = curverad[2]

        font = cv2.FONT_HERSHEY_SIMPLEX
        fontColor = (125, 255, 125)
        fontSize = 1
        if lane_curve is None:
            lane_curve = 0
        if offset is None:
            offset = 0
        cv2.putText(lanes, 'Lane Curvature: {:.0f} m'.format(lane_curve), (100, 400), font, fontSize, fontColor, 2)
        cv2.putText(lanes, 'Vehicle offset: {:.2f} m'.format(offset), (100, 450), font, fontSize, fontColor, 2)
        return lanes


if __name__ == "__main__":
    # Straights
    img = cv2.imread('resources/straight_1.jpg')  # ok
    img = cv2.imread('resources/straight_2.jpg')  # good
    img = cv2.imread('resources/straight_3.jpg')  # good
    # img = cv2.imread('resources/straight_4.jpg')  # good

    # Curves
    img = cv2.imread('resources/curve_1.jpg')  # left (good)
    img = cv2.imread('resources/curve_2.jpg')  # left (ok)
    img = cv2.imread('resources/curve_3.jpg')  # right (ok)
    img = cv2.imread('resources/curve_4.jpg')  # right (good)

    # Offset
    img = cv2.imread('resources/offset_1.jpg')  # too far
    img = cv2.imread('resources/offset_2.jpg')  # just right
    img = cv2.imread('resources/offset_3.jpg')  # too close

    # Rotation
    img = cv2.imread('resources/rotation_1.jpg')  # original
    img = cv2.imread('resources/rotation_2.jpg')  # 10deg left
    img = cv2.imread('resources/rotation_3.jpg')  # 10deg right
    # img = cv2.imread('resources/rotation_4.jpg')# 8deg right (curve)

    # Edge Cases / Reject
    img = cv2.imread('resources/edgecase_1.jpg')  # reject
    # img = cv2.imread('resources/edgecase_2.jpg')         # reject (no lane in scope)
    # img = cv2.imread('resources/edgecase_3.jpg')         # no grass (lucky detect)
    img = cv2.imread('resources/edgecase_4.jpg')  # crossing
    img = cv2.imread('resources/edgecase_5.jpg')  # incorrect detection
    # img = cv2.imread('resources/edgecase_6.jpg')         # out of bounds (camera angle too small)

    secondOrderEdgeRecognizer = SecondOrderEdgeRecognizer(debug=True, destination_size=(800, 600))
    for i in range(1):
        print("Processing result:", secondOrderEdgeRecognizer.process(img))
    curverad = secondOrderEdgeRecognizer.get_curve_radius()
    print(curverad)
    result_img = secondOrderEdgeRecognizer.visualize_lane()
    plt.imshow(result_img, cmap="hsv")
    plt.show()
