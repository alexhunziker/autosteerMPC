class Parameters(object):
    def __init__(self):
        self.ultrasonic = 500
        self.ultrasonic_timestamp = None

        self.gps = {"lat": None, "lon": None, "altitude": None, "speed": None}
        self.gps_timestamp = None

        self.yaw_rate = None
        self.yaw_rate_timestamp = None

        self.lidar = None
        self.lidar_timestamp = None

        self.speed = 0
        self.speed_timestamp = None
