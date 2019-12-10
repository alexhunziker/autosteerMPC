import math
import time

class Parameters(object):
    def __init__(self):
        self.distance = 500
        self.distance_timestamp = time.time()

        self.gps = {"lat": 0, "lon": 0, "altitude": None, "speed": None, "yaw": None, "yaw_rate": None}
        self.gps_timestamp = time.time()

        self.next_target = (0, 0)

        self.lane_curvature = None
        self.lateral_offset = None
        self.cv_timestamp = time.time()

        self.yaw_rate = None

        self.yaw_target = 0
        self.yaw_target_timestamp = time.time()

    # TODO: Duplication, this is also done in MPCBridge, however it should be in neither of these places
    # move to SensorFuser
    def update_target_yaw(self): 
        try:
            self.yaw_target = math.atan2(self.gps["lon"] - self.next_target[1], self.gps["lat"] - self.next_target[0])
            self.yaw_target_timestamp = time.time()
        except:
            print("ERR: Updating target yaw failed. Next target is", self.next_target, " and gps data is", self.gps)

    def __str__(self):
        state_string = "Distance " + str(self.distance) + "cm" + "\n"
        state_string += "GPS " + str(self.gps) + "\n"
        state_string += "Next waypoint " + str(self.next_target) + "\n"
        state_string += "Lane curvature " + str(self.lane_curvature) + "\n"
        state_string += "Lateral offset " + str(self.lateral_offset) + "\n"
        state_string += "speed " + str(self.gps["speed"]) + "m/s" + "\n"
        return state_string


