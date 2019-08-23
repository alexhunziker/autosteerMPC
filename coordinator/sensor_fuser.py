from parameters import Parameters

from camera import camera
from cv import lane_recognizer
from gps_sensor import gps_sensor
from ultrasonic import ultrasonic_sensor


class SensorFuser(object):
    def __init__(self):
        self.ultrasonic = ultrasonic_sensor.UltrasonicSensor()
        self.gps = gps_sensor.GPSSensor()
        self.camera = camera.Camera()
        self.lane_recognizer = lane_recognizer.LaneRecognizer(self.camera)

    def retrieve_updates(self):
        parameters = Parameters()

        parameters.ultrasonic = self.ultrasonic.retrieve_state()
        parameters.ultrasonic_timestamp = self.ultrasonic.last_valid

        parameters.gps = self.gps.retrieve_state()
        parameters.gps_timestamp = self.gps.last_valid

        parameters.lane_curvature, parameters.lateral_offset = self.lane_recognizer.retrieve_state()
        parameters.cv_timestamp = self.lane_recognizer.last_valid

        return parameters
