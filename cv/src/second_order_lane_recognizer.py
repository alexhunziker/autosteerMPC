import cv2
import matplotlib.pyplot as plt

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
        edges_image = self.image_preprocessor.process(img)
        warped_image = self.image_warper.warp(edges_image)
        self.lane_recognizer.sliding_window(warped_image)
        self.img = img
        return self

    def get_curve_radius(self):
        return self.lane_recognizer.fit_curve_worldspace(self.img)

    def visualize_lane(self):
        lanes = self.lane_recognizer.draw_lanes(self.img)
        plt.imshow(lanes, cmap="hsv")
        plt.show()


img = cv2.imread('resources/curve_1.jpg')
secondOrderLaneRecognizer = SecondOrderLaneRecognizer().process(img)
curverad = secondOrderLaneRecognizer.get_curve_radius()
print(curverad)
secondOrderLaneRecognizer.visualize_lane()
# img_ = lane_recognizer.draw_lanes(img)
# plt.imshow(img_, cmap='hsv')
# plt.show()
