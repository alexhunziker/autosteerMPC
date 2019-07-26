import time

from actuator_bridge.py import ActuatorBridge
from health_checker.py import HealthChecker
from mpc_bridge.py import MPCBridge
from path_manager.py import PathManager
from sensor_fuser.py import SensorFuser


class Coordinator(object):
    STEP_TIME = 0.1

    def __init__(self):
        self.path_manager = PathManager()
        self.sensor_fuser = SensorFuser()
        self.health_checker = HealthChecker()
        self.mpc_bridge = MPCBridge()
        self.actuator_bridge = ActuatorBridge()
        self.active = True

    def stop_system(self):
        self.active = False

    def start_trip(self, start, destination):
        self.path_manager.retrieve_path(start, destination)
        while self.active:
            self.main_loop()
            time.sleep(Coordinator.STEP_TIME)

    def main_loop(self):
        parameters = self.sensor_fuser.retrieve_updates()
        self.path_manager.potentially_update_next(parameters.gps)
        self.health_checker.check(parameters)
        impulses = self.mpc_bridge.request_step(parameters)
        self.actuator_bridge.send(impulses)
