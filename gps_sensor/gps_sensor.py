import threading
import time

import gps


class GPSSensor(object):
    def __init__(self, verbose=False):
        self.stop = False
        self.verbose = verbose
        self.last_valid = time.time()
        self.gps = {"lat": 0, "lon": 0, "altitude": 0, "speed": 0}

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
                "speed": gpsd.fix.speed
            }
            if self.gps["lat"] > 0:
                self.gps = gps_received
                self.last_valid = time.time()
            if self.verbose:
                print(self.gps)
            time.sleep(0.1)
            gpsd.next()

    def stop_measuring(self):
        self.stop = True

    def retrieve_state(self):
        return self.gps


if __name__ == "__main__":
    GPSSensor(verbose=True)
