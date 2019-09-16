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
        
        self.lidar = ctypes.CDLL("../lidar/lidar.so")
        self.lidar.getDistanceForAngle.argtypes = (ctypes.c_double)
        # TODO: Return type required

    def retrieve_updates(self):
        parameters = Parameters()

        self.getDistance()

        parameters.gps = self.gps.retrieve_state()
        parameters.gps_timestamp = self.gps.last_valid

        parameters.lane_curvature, parameters.lateral_offset = self.lane_recognizer.retrieve_state()
        parameters.cv_timestamp = self.lane_recognizer.last_valid

        return parameters

    def get_distance(self):
        ultrasonic_dist = self.ultrasonic.retrieve_state()
        lidar_dist = self.lidar.getDistanceForAngle(ctypes.c_double(0))
        if ultrasonic_dist > 500:
            parameters.ultrasonic = lidar_dist      # TODO: RENAME
        else:
            parameters.ultrasonic = min(lidar_dist, ultrasonic_dist)
        parameters.ultrasonic_timestamp = min(
            self.ultrasonic.last_valid,
            self.lidar.getLastValidForAngle(ctypes.c_double(0))
        )

    def correct_yaw_if_blocked(self):   # TODO: Does not belong here
        distance_target_yaw_direction = self.lidar.getDistanceForAngle(ctypess.c_double(parameters.yaw_target))
        if parameters.speed / parameters.ultrasonic > 3:
            return
        
        probe_angle = parameters.yaw_target
        while(abs(parameters.yaw_target) > 0):
            current_distance = self.lidar.getDistanceForAngle(ctypess.c_double(parameters.probe_angle))
            if self.lidar.getLastValidForAngle(ctypess.c_double(parameters.probe_angle))<CUTOFF and (current_distance>3 or current_distance > distance_target_yaw_direction*1.5): # TODO: Las condition sensible?  also define cutoff
                parameters.yaw_target = probe_angle
                break
            if(parameters.yaw_target) > 0:
                probe_angle -= 0.02
            else:
                probe_angle += 0.02
        parameters.target_speed = getSpeedForAngle(probe_angle)
