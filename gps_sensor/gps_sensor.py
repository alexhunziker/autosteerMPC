import threading
import time
import statistics
import math
import copy

import gps


class GPSSensor(object):
    HISTORY_THRESHOLD = 2
    
    def __init__(self, verbose=False, averaging=False):
        self.stop = False
        self.verbose = verbose
        self.averaging = averaging
        self.history = []
        self.last_valid = time.time()
        self.gps = {"lat": 0, "lon": 0, "altitude": 0, "speed": 0, "yaw": 0, "yaw_rate": 0}
        self.last_raw_yaw = 0

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
            # invalidate bad measurements
            if gpsd.pdop>5 or gpsd.pdop<1:
                gps_received["lat"] = 0
                gps_received["lon"] = 0
                gps_received["yaw"] = 0
                gps_received["speed"] = 0
                print("WARN: GPS measurement inaccurate and therefore rejected.")
            gps_received = self.satitize_measurements(gps_received)
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
        return copy.deepcopy(self.gps)

    def satitize_measurements(self, current_state):
        print("DEBUG: current GPS measurement is", current_state)

        # Replace nan with 0
        if math.isnan(current_state["speed"]):
            current_state["speed"] = 0
            print("WARN: No speed recieved, assumed 0")
        if math.isnan(current_state["yaw"]):
            current_state["yaw"] = 0
            current_state["yaw_rate"] = 0
            print("ERROR: No yaw (track) recieved, assumed 0")

        # Only use a non-zero yaw rate if last and current measurement are non-zero
        if self.last_raw_yaw == 0.0 or current_state["yaw"]==0.0:
            current_state["yaw_rate"] = 0
        self.last_raw_yaw = current_state["yaw"]

        return current_state

    def average_measurements(self, current_state):

        # Take previous yaw if no yaw recieved (happens at low speeds)
        if current_state["yaw"] == 0.0 and len(self.history)>0:
            current_state["yaw"] = self.history[-1]["yaw"]
        
        # Reset history if it is outdated (sensor was down) or no history is present
        if self.history == [] or (time.time()-self.last_valid)>GPSSensor.HISTORY_THRESHOLD:
            if self.verbose:
                print("DEBUG: GPS history reset")
            self.history = [current_state, current_state, current_state]
        else:
            self.history.pop(0)
            self.history.append(current_state)  # here we determmine size of history

        # Average speed and yaw
        current_state["speed"] = statistics.mean(map(lambda x: x["speed"], self.history))
        current_state["yaw"] = statistics.mean(map(lambda x: x["yaw"], self.history))
        
        return current_state

if __name__ == "__main__":
    GPSSensor(verbose=True)
