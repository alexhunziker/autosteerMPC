import multiprocessing
from cv.lane_recognizer import LaneRecognizer
from camera.camera import Camera

class CVProcess(multiprocessing.Process):
    def __init__(self, results):
        self.results = results
        self.camera = Camera(verbose=True)
        self.lane_recognizer = LaneRecognizer(self.camera)
    
    def measure(self):
        self.results[0] = self.lane_recognizer.curve_radius
        self.results[1] = self.lane_recognizer.lateral_deviation
        self.results[2] = self.lane_recognizer.last_valid

    def stop(self):
        self.camera.stop_measuring()
        self.lane_recognizer.stop_measuring()