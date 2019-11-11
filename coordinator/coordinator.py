import time
import threading

from actuator_bridge import ActuatorBridge
from health_checker import HealthChecker
from mpc_bridge import MPCBridge
from path_manager import PathManager
from sensor_fuser import SensorFuser


class Coordinator(object):
    STEP_TIME = 0.1

    def __init__(self, mock_actuator=False):
        self.health_checker = HealthChecker()
        self.health_checker.double_flash()
        self.path_manager = PathManager()
        self.mpc_bridge = MPCBridge()
        self.actuator_bridge = ActuatorBridge(mock=mock_actuator)
        self.health_checker.double_flash()

        self.active = True

    def stop_system(self):
        self.active = False

    def start_trip(self, start, destination, use_cv=True, fake_gps=False, speed=None):
        self.health_checker.startup()
        self.health_checker.double_flash()
        self.path_manager.retrieve_path(start, destination)
        self.sensor_fuser = SensorFuser(use_cv=use_cv, fake_gps=fake_gps, speed=speed)
        coordinator_thread = threading.Thread(target=self.main_loop)
        coordinator_thread.start()
        self.health_checker.startup_done()

    def main_loop(self):
        while self.active:
            loop_start = time.time()
            parameters = self.sensor_fuser.retrieve_updates()
            self.path_manager.potentially_update_next(parameters.gps)
            parameters.next_target = self.path_manager.get_next()
            self.health_checker.check(parameters)
            impulses = self.mpc_bridge.request_step(parameters)
            self.actuator_bridge.send(impulses)

            sleep_time = Coordinator.STEP_TIME - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)
                print("DEBUG: Main loop took", 0.1 - sleep_time)
            else:
                print("WARN: Main loop took too long to process: ", 0.1 - sleep_time)
        self.sensor_fuser.stop()


if __name__ == "__main__":
    coordinator = Coordinator(mock_actuator=True)
    coordinator.start_trip("0x0", "0x3")
    time.sleep(20)
    #coordinator.stop_system()

