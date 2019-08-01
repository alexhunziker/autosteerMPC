from parameters.py import Parameters

from gps_sensor import gps_sensor
from ultrasonic import ultrasonic_sensor


class SensorFuser(object):
    def __init__(self):
        self.ultrasonic = ultrasonic_sensor.UltrasonicSensor()
        self.gps = gps_sensor.GPSSensor()

    def retrieve_updates(self):
        parameters = Parameters()

        parameters.ultrasonic = self.ultrasonic.retrieve_state()
        parameters.ultrasonic_timestamp = self.ultrasonic.last_valid

        parameters.gps = self.gps.retrieve_state()[0]
        parameters.gps_timestamp = self.gps.last_valid

        return parameters
