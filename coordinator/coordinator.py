import time
import threading
import datetime
import sys
import os

from actuator_bridge import ActuatorBridge
from health_checker import HealthChecker
from mpc_bridge import MPCBridge
from path_manager import PathManager
from sensor_fuser import SensorFuser
from impulses import Impulses


class Coordinator(object):
    STEP_TIME = 0.2

    def __init__(self, mock_actuator=False):
        self.health_checker = HealthChecker()
        self.health_checker.double_flash()
        self.path_manager = PathManager(verbose=True)
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
        time.sleep(2)       # Wait until Arduino is ready
        while self.active:
            print("INFO: System time is", str(datetime.datetime.now()))
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
                print("DEBUG: Main loop took", 0.2 - sleep_time)
            else:
                print("WARN: Main loop took long to process: ", 0.2 - sleep_time)
        self.sensor_fuser.stop()


if __name__ == "__main__":
    mock_actuator=False
    coordinator = Coordinator(mock_actuator=mock_actuator)
    coordinator.start_trip("0xc", "0xd", use_cv=False)     # Route tracking
    #coordinator.start_trip("0x6", "0x7", use_cv=False)      # Obstacle
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("INFO: Shutting down")
        ActuatorBridge(mock=mock_actuator).send(Impulses(0, 0, 1))
        coordinator.stop_system()
        for x in range(5):
            time.sleep(0.2)
            ActuatorBridge(mock=mock_actuator).send(Impulses(0, 0, 1))
        time.sleep(0.2)
        ActuatorBridge(mock=mock_actuator).send(Impulses(0, 0, 0))
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)

