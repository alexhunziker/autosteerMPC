import threading
import time
import statistics
import math

import gps


class GPSSensor(object):
    HISTORY_THRESHOLD = 2
    
    def __init__(self, verbose=False, averaging=True):
        self.stop = False
        self.verbose = verbose
        self.averaging = averaging
        self.history = []
        self.last_valid = time.time()
        self.gps = {"lat": 0, "lon": 0, "altitude": 0, "speed": 0, "yaw": 0, "yaw_rate": 0}

        measure_thread = threading.Thread(target=self.measure_loop)
        measure_thread.start()

    def measure_loop(self):
        gpsd = gps.gps(mode=gps.WATCH_ENABLE)
        while True:
            if self.stop:
                break
            gps_received = {
                "lat": gpsd.fix.latitude,
                "lon": gpsd.fix.longitude,
                "altitude": gpsd.fix.altitude,
                "speed": gpsd.fix.speed,
                "yaw": gpsd.fix.track,
                "yaw_rate": gpsd.fix.track - self.gps["yaw"]
            }
            if self.averaging:
                gps_received = self.average_measurements(gps_received)
            if gps_received["lat"] > 0:
                self.gps = gps_received
                self.last_valid = time.time()
            if self.verbose:
                print("DEBUG: GPS State is", self.gps)
            time.sleep(0.1)
            gpsd.next()

    def stop_measuring(self):
        self.stop = True

    def retrieve_state(self):
        return self.gps

    def average_measurements(self, current_state):
        if self.history == [] or (time.time()-self.last_valid)>GPSSensor.HISTORY_THRESHOLD:
            if self.verbose:
                print("DEBUG: GPS history reset")
            self.history = [current_state, current_state, current_state]
        else:
            self.history.pop(0)
            self.history.append(current_state)

        if math.isnan(current_state["speed"]):
            current_state["speed"] = 0
            print("WARN: No speed recieved, assumed 0")
        if math.isnan(current_state["yaw"]):
            current_state["yaw"] = 0
            current_state["yaw_rate"] = 0
            print("ERROR: No yaw (track) recieved, assumed 0")
        current_state["speed"] = statistics.mean(map(lambda x: x["speed"], self.history))
        current_state["yaw"] = statistics.mean(map(lambda x: x["yaw"], self.history))
        current_state["yaw_rate"] = statistics.mean(map(lambda x: x["yaw_rate"], self.history))
        return current_state

if __name__ == "__main__":
    GPSSensor(verbose=True)
