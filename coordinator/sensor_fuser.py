import ctypes
import sys
import time
import threading

sys.path.append('../')

from parameters import Parameters
from camera.camera import Camera
from cv.lane_recognizer import LaneRecognizer
from ultrasonic.ultrasonic_sensor import UltrasonicSensor
from gps_sensor.gps_sensor import GPSSensor


class SensorFuser(object):
    def __init__(self, verbose=True):
        self.verbose = verbose
        self.ultrasonic = UltrasonicSensor(verbose=False)
        self.gps = GPSSensor(verbose=False)
        self.camera = Camera(verbose=False)
        self.lane_recognizer = LaneRecognizer(self.camera)
        
        self.lidar = ctypes.CDLL("../lidar/build/liblidar_sensor.so")
        self.lidar.getDistanceForAngle.argtypes = ([ctypes.c_double])
        self.lidar.getDistanceForAngle.restype = ctypes.c_double
        self.lidar.getLastValidForAngle.argtypes = ([ctypes.c_double])
        self.lidar.getLastValidForAngle.restype = ctypes.c_double
        measure_thread = threading.Thread(target=self.lidar_loop)
        measure_thread.start()

    def lidar_loop(self):
        self.lidar.main()

    def retrieve_updates(self):
        parameters = Parameters()

        self.get_distance(parameters)

        parameters.gps = self.gps.retrieve_state()
        parameters.gps_timestamp = self.gps.last_valid

        parameters.lane_curvature, parameters.lateral_offset = self.lane_recognizer.retrieve_state()
        parameters.cv_timestamp = self.lane_recognizer.last_valid

        return parameters

    def stop(self):
        self.ultrasonic.stop_measuring()
        self.gps.stop_measuring()
        self.lidar.stop_measuring()
        self.camera.stop_measuring()
        self.lane_recognizer.stop_measuring()

    def get_distance(self, parameters):
        ultrasonic_dist = self.ultrasonic.retrieve_state()
        lidar_dist = self.lidar.getDistanceForAngle(ctypes.c_double(0))-5
        if ultrasonic_dist > 500:
            parameters.distance = lidar_dist      # TODO: RENAME
        else:
            parameters.distance = min(lidar_dist, ultrasonic_dist)
            if lidar_dist < ultrasonic_dist:
                if(self.verbose):
                    print("DEBUG: Taking lidar distance of ", lidar_dist)
                parameters.distance = lidar_dist
                parameters.distance_timestamp = self.lidar.getLastValidForAngle(0)
            else: 
                if(self.verbose):
                    print("DEBUG: Taking ultrasonic distance of ", ultrasonic_dist)
                parameters.distance = ultrasonic_dist
                parameters.distance_timestamp = self.ultrasonic.last_valid

    #def correct_yaw_if_blocked(self):   # TODO: Does not belong here
    #    distance_target_yaw_direction = self.lidar.getDistanceForAngle(ctypess.c_double(parameters.yaw_target))
    #    if parameters.speed / parameters.ultrasonic > 3:
    #        return
    #    
    #    probe_angle = parameters.yaw_target
    #    while(abs(parameters.yaw_target) > 0):
    #        current_distance = self.lidar.getDistanceForAngle(ctypes.c_double(parameters.probe_angle))
    #        if self.lidar.getLastValidForAngle(ctypess.c_double(parameters.probe_angle))<CUTOFF and (current_distance>3 or current_distance > distance_target_yaw_direction*1.5): # TODO: Las condition sensible?  also define cutoff
    #            parameters.yaw_target = probe_angle
    #            break
    #        if(parameters.yaw_target) > 0:
    #            probe_angle -= 0.02
    #        else:
    #            probe_angle += 0.02
    #    parameters.target_speed = getSpeedForAngle(probe_angle)


if __name__ == "__main__":
        fuser = SensorFuser()
        time.sleep(10)
        parameters = fuser.retrieve_updates()
        fuser.stop()
        print(parameters)
