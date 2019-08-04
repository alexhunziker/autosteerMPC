class Parameters(object):
    def __init__(self):
        self.ultrasonic = 500
        self.ultrasonic_timestamp = None
        self.gps = {"lat": None, "lon": None, "altitude": None, "speed": None}
        self.gps_timestamp = None
        self.lidar = None
        self.curvature_yaw = None
