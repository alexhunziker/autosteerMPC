import ctypes
import sys
import time
import threading
import multiprocessing
import math
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

    def __init__(self, verbose=True, use_cv=True, fake_gps=False, speed=None):
        self.verbose = verbose
        self.use_cv = use_cv
        self.fake_gps = fake_gps
        self.ultrasonic = UltrasonicSensor(verbose=False)

        if not fake_gps:
            self.gps = GPSSensor(verbose=True)
        else:
            self.fake_speed = speed

        self.cv_results = multiprocessing.Array("d", [float("nan"), float("nan"), float("nan")])
        if self.use_cv:
            self.cv_worker = CVProcess(self.cv_results)
            self.cv_worker.start()

        self.lidar = ctypes.CDLL("../lidar/build/liblidar_sensor.so")
        self.lidar.getDistanceForAngle.argtypes = ([ctypes.c_double])
        self.lidar.getDistanceForAngle.restype = ctypes.c_double
        self.lidar.getLastValidForAngle.argtypes = ([ctypes.c_double])
        self.lidar.getLastValidForAngle.restype = ctypes.c_double
        self.lidar.disableDebug()
        measure_thread = threading.Thread(target=self.lidar_loop)
        measure_thread.start()

    def lidar_loop(self):
        self.lidar.main()

    def retrieve_updates(self):
        parameters = Parameters()
        parameters.update_target_yaw()

        self.get_distance(parameters)
        self.adjust_distance_for_target_direction(parameters)

        if not self.fake_gps:
            parameters.gps = self.gps.retrieve_state()
            parameters.gps_timestamp = self.gps.last_valid
        else:
            parameters.gps["speed"] = self.fake_speed

        if self.use_cv:
            self.cv_worker.retrieve_state()
        parameters.lane_curvature = self.cv_results[0]
        parameters.lateral_offset = self.cv_results[1]
        parameters.cv_timestamp = self.cv_results[2]

        return parameters

    def stop(self):
        self.ultrasonic.stop_measuring()
        self.gps.stop_measuring()
        self.lidar.stop_measuring()
        try:
            self.cv_worker.stop()
        except:
            print("WARN: CV worker could not be stopped")
        #self.cv_worker.join()

    def get_distance(self, parameters):
        ultrasonic_dist = self.ultrasonic.retrieve_state()
        lidar_dist = self.lidar.getDistanceForAngle(ctypes.c_double(0))-5
        lidar_time = self.lidar.getLastValidForAngle(0)
        # If Lidar distance is 0, this means it usually means no obstacle detected
        if lidar_dist == 0.0:
            lidar_dist = 1_000

        print("DEBUG: Lidar and ultrasonic dist", lidar_dist, ultrasonic_dist)
        if ultrasonic_dist is None and time.time()-lidar_time>1.0:          # No valid measurements
            print("WARN: No reliable distance information, assuming 0")
            parameters.distance = 0
        elif ultrasonic_dist > 450 and (lidar_dist > 1200 or time.time()-lidar_time>1.0):  # No obstacle detected
            print("INFO: No obstacle detected")
            print("INFO: lidar time diff", time.time()-lidar_time, "with", time.time(), lidar_time)
            parameters.distance_timestamp = time.time()
            parameters.distance = 1_200
        elif ultrasonic_dist is None or ultrasonic_dist > 450:              # Only lidar signal valid
            print("INFO: Lidar distance only of", lidar_dist)
            parameters.distance = lidar_dist
            parameters.distance_timestamp = lidar_time
        elif time.time()-lidar_time>1.0:                                    # Only ultrasonic signal valid
            print("INFO: Ultrasonic distance only of", ultrasonic_dist)
            parameters.distance = ultrasonic_dist
            parameters.distance_timestamp = self.ultrasonic.last_valid
        else:                                                               # Both sensors valid
            parameters.distance = min(lidar_dist, ultrasonic_dist)
            if lidar_dist < ultrasonic_dist:
                if(self.verbose or True):
                    print("DEBUG: Taking lidar distance of ", lidar_dist)
                parameters.distance_timestamp = self.lidar.getLastValidForAngle(0)
            else:
                if(self.verbose or True):
                    print("DEBUG: Taking ultrasonic distance of ", ultrasonic_dist)
                parameters.distance_timestamp = self.ultrasonic.last_valid
        
    def adjust_distance_for_target_direction(self, parameters):
        distance_target_yaw_direction = self.lidar.getDistanceForAngle(
            ctypes.c_double(parameters.yaw_target))
        if parameters.gps["speed"] / (distance_target_yaw_direction+SensorFuser.EPS) > 3:
            return

        yaw_target = parameters.yaw_target if parameters.yaw_target<math.pi else -2*math.pi + parameters.yaw_target
        probe_angle = parameters.yaw_target
        while(abs(probe_angle) > 0.4):
            probe_distance_coll = parameters.gps["speed"] / \
                self.lidar.getDistanceForAngle(ctypes.c_double(parameters.probe_angle))
            if time.time()-self.lidar.getLastValidForAngle(ctypes.c_double(parameters.probe_angle)) < self.LIDAR_CUTOFF and (probe_distance_coll > 3 or probe_distance_coll > distance_target_yaw_direction*1.5):  # TODO: Las condition sensible?  also define cutoff
                parameters.yaw_target = probe_angle
                print("INFO: Yaw target corrected by 360 lidar to", parameters.yaw_target)
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
