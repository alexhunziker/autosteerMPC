import threading
import time

import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray

from .second_order_lane_recognizer import SecondOrderLaneRecognizer


class LaneRecognizer(object):
    RESOLUTION = (1280, 720)

    def __init__(self, debug=False):
        self.debug = debug
        self.camera = PiCamera()
        self.camera.resolution = LaneRecognizer.RESOLUTION
        self.camera.rotation = 180
        self.raw_capture = PiRGBArray(self.camera)
        self.curve_radius = None
        self.lateral_deviation = None
        self.last_valid = None
        self.stop = False
        self.second_order_recognizer = SecondOrderLaneRecognizer(destination_size=LaneRecognizer.RESOLUTION,
                                                                 debug=self.debug)
        measure_thread = threading.Thread(target=self.measure_loop)
        measure_thread.start()

    def measure_loop(self):
        time.sleep(1)
        while not self.stop:
            self.target_yaw_rate = self.analyze_frame()

    def stop(self):
        self.stop = True

    def analyze_frame(self):
        # TODO: Move to camera module, for architecture but also for performance.
        try:
            self.raw_capture = PiRGBArray(self.camera)
            self.camera.capture(self.raw_capture, format="bgr")
        except:
            print("WARN: Image capturing failed...")
            time.sleep(0.1)
            return None

        try:
            image = self.raw_capture.array
            self.curve_radius, self.lateral_deviation = self.second_order_recognizer.process(image)
        except:
            print("WARN: Image processing failed...")
            time.sleep(0.1)
            return None

        if self.debug:
            result = self.second_order_recognizer.visualize_lane()
            cv2.imshow('result', result)
            cv2.waitKey(0)

        self.last_valid = time.time()
        return 1

    def retrieve_state(self):
        return self.curve_radius, self.lateral_deviation


if __name__ == "__main__":
    LaneRecognizer(debug=True)
