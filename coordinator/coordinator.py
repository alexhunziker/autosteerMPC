import time
import threading

from actuator_bridge import ActuatorBridge
from health_checker import HealthChecker
from mpc_bridge import MPCBridge
from path_manager import PathManager
from sensor_fuser import SensorFuser


class Coordinator(object):
    STEP_TIME = 0.1

    def __init__(self):
        self.health_checker = HealthChecker()
        self.health_checker.double_flash()

        self.path_manager = PathManager()
        self.sensor_fuser = SensorFuser()
        self.mpc_bridge = MPCBridge()
        self.actuator_bridge = ActuatorBridge()

        self.active = True
        self.health_checker.ready()

    def stop_system(self):
        self.active = False

    def start_trip(self, start, destination):
        self.health_checker.double_flash()
        self.path_manager.retrieve_path(start, destination)
        coordinator_thread = threading.Thread(target=self.main_loop)
        coordinator_thread.start()

    def main_loop(self):
        while self.active:
            parameters = self.sensor_fuser.retrieve_updates()
            self.path_manager.potentially_update_next(parameters.gps)
            parameters.next_target = self.path_manager.get_next()
            self.health_checker.check(parameters)
            impulses = self.mpc_bridge.request_step(parameters)
            self.actuator_bridge.send(impulses)
            time.sleep(Coordinator.STEP_TIME)
        self.sensor_fuser.stop()


if __name__ == "__main__":
    coordinator = Coordinator()
    coordinator.start_trip("0xa", "0xab")
    time.sleep(20)
    coordinator.stop_system()

