import threading
import time
import traceback

import cv2

from .second_order_lane_recognizer import SecondOrderLaneRecognizer


class LaneRecognizer(object):
    RESOLUTION = (800, 600)

    def __init__(self, camera, debug=False, mode="edges"):
        self.debug = debug
        self.camera_object = camera
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

    def stop_measuring(self):
        self.stop = True

    def analyze_frame(self):
        try:
            image = self.camera_object.retrieve_data()
            if image is None:
                print("WARN: Image data cannot be retrieved from camera module")
                return None
            self.curve_radius, self.lateral_deviation = self.second_order_recognizer.process(image)
        except:
            print("WARN: Image processing failed...")
            traceback.print_exc()
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
