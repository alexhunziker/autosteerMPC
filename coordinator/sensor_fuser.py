from parameters.py import Parameters

from ultrasonic import UltrasonicSensor


class SensorFuser(object):
    def __init__(self):
        self.ultrasonic = UltrasonicSensor

    def retrieve_updates(self):
        parameters = Parameters()
        parameters.ultrasonic = self.ultrasonic.retrieve_state()

        return parameters
