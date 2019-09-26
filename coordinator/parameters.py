class Parameters(object):
    def __init__(self):
        self.distance = 500
        self.distance_timestamp = None

        self.gps = {"lat": None, "lon": None, "altitude": None, "speed": None}
        self.gps_timestamp = None

        self.next_target = None

        self.lane_curvature = None
        self.lateral_offset = None
        self.cv_timestamp = None

        self.yaw_rate = None

        self.speed = 0
        self.speed_timestamp = None

    def __str__(self):
        state_string = "Distance " + str(self.distance) + "cm" + "\n"
        state_string += "GPS " + str(self.gps) + "\n"
        state_string += "Next waypoint " + str(self.next_target) + "\n"
        state_string += "Lane curvature " + str(self.lane_curvature) + "\n"
        state_string += "Lateral offset " + str(self.lateral_offset) + "\n"
        state_string += "speed " + str(self.speed) + "m/s" + "\n"
        state_string += "yaw rate " + str(self.speed) + "rad/s" + "\n"
        return state_string


