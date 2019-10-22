import ctypes
import sys
import time
import threading
import multiprocessing
sys.path.append('../')

from gps_sensor.gps_sensor import GPSSensor
from ultrasonic.ultrasonic_sensor import UltrasonicSensor
from cv.lane_recognizer import LaneRecognizer
from camera.camera import Camera
from parameters import Parameters
from cv_process import CVProcess

class SensorFuser(object):
    LIDAR_CUTOFF = 2.0
    EPS = 0.000001

    def __init__(self, verbose=True):
        self.verbose = verbose
        self.ultrasonic = UltrasonicSensor(verbose=False)
        self.gps = GPSSensor(verbose=True)
        self.cv_results = multiprocessing.Array("d", [float("nan"), float("nan"), float("nan")])
        self.cv_worker = CVProcess(self.cv_results)
        self.cv_worker.start()

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
        parameters.update_target_yaw()

        self.get_distance(parameters)
        self.adjust_distance_for_target_direction(parameters)

        parameters.gps = self.gps.retrieve_state()
        parameters.gps_timestamp = self.gps.last_valid

        self.cv_worker.retrieve_state()
        parameters.lane_curvature = self.cv_results[0]
        parameters.lateral_offset = self.cv_results[1]
        parameters.cv_timestamp = self.cv_results[2]

        return parameters

    def stop(self):
        self.ultrasonic.stop_measuring()
        self.gps.stop_measuring()
        self.lidar.stop_measuring()
        self.cv_worker.stop()
        #self.cv_worker.join()

    def get_distance(self, parameters):
        ultrasonic_dist = self.ultrasonic.retrieve_state()
        lidar_dist = self.lidar.getDistanceForAngle(ctypes.c_double(0))-5
        if ultrasonic_dist > 500:
            parameters.distance = lidar_dist
            parameters.distance_timestamp = self.ultrasonic.last_valid
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

    # TODO: LIDAR class, let's see if this makes sense. Also GPS version only a ce moment la.
    def adjust_distance_for_target_direction(self, parameters):
        distance_target_yaw_direction = self.lidar.getDistanceForAngle(
            ctypes.c_double(parameters.yaw_target))
        if parameters.speed / (distance_target_yaw_direction+SensorFuser.EPS) > 3:
            return

        probe_angle = parameters.yaw_target
        while(abs(probe_angle) > 0.4):
            probe_distance_coll = parameters.speed / \
                self.lidar.getDistanceForAngle(ctypes.c_double(parameters.probe_angle))
            if time.time()-self.lidar.getLastValidForAngle(ctypes.c_double(parameters.probe_angle)) < self.LIDAR_CUTOFF and (probe_distance_coll > 3 or probe_distance_coll > distance_target_yaw_direction*1.5):  # TODO: Las condition sensible?  also define cutoff
                parameters.yaw_target = probe_angle
                break
            if(parameters.yaw_target) > 0:
                probe_angle -= 0.02
            else:
                probe_angle += 0.02


if __name__ == "__main__":
    fuser = SensorFuser()
    time.sleep(10)
    parameters = fuser.retrieve_updates()
    fuser.stop()
    print(parameters)
