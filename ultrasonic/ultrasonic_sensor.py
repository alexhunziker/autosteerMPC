import threading
import time

import RPi.GPIO as GPIO


class UltrasonicSensor(object):
    GPIO_PIN = 4
    TIMEOUT_1 = 1000
    TIMEOUT_2 = 50000

    usleep = lambda x: time.sleep(x / 1_000_000.0)

    def __init__(self, verbose=False):
        GPIO.setmode(GPIO.BCM)
        self.stop = False
        self.verbose = verbose
        self.last_valid = time.time()
        self.measured_distance = None
        measure_thread = threading.Thread(target=self.measure_loop)
        measure_thread.start()

    def measure_loop(self):
        time.sleep(1)
        while not self.stop:
            self.measured_distance = self.measure()
            UltrasonicSensor.usleep(100_000)
            if self.verbose:
                print("Distance is", self.measured_distance, "cm")

    def measure(self):
        self.trigger_signal()
        measurement = self.receive_signal()
        distance = self.calculate_distance_cm(measurement)
        if distance > 500:
            if self.verbose:
                print("DEBUG: Ultrasonic distance out of bounds, assuming max value")
            distance = 500
        return distance

    def trigger_signal(self):
        GPIO.setup(UltrasonicSensor.GPIO_PIN, GPIO.OUT)
        GPIO.output(UltrasonicSensor.GPIO_PIN, GPIO.HIGH)
        UltrasonicSensor.usleep(2)
        GPIO.output(UltrasonicSensor.GPIO_PIN, GPIO.LOW)
        UltrasonicSensor.usleep(10)
        GPIO.output(UltrasonicSensor.GPIO_PIN, GPIO.HIGH)

    def receive_signal(self):
        t_0 = time.time()
        GPIO.setup(UltrasonicSensor.GPIO_PIN, GPIO.IN)
        for count in range(UltrasonicSensor.TIMEOUT_1):
            if GPIO.input(UltrasonicSensor.GPIO_PIN):
                break
        if count >= UltrasonicSensor.TIMEOUT_1:
            print("M1 failed")
            return None
        t_1 = time.time()
        for count in range(UltrasonicSensor.TIMEOUT_2):
            if not GPIO.input(UltrasonicSensor.GPIO_PIN):
                break
        if count >= UltrasonicSensor.TIMEOUT_2-1:
            print("ERROR: Ultrasonic M2 failed")
            return None
        self.last_valid = time.time()
        t_2 = time.time()
        return t_2 - t_1

    def calculate_distance_cm(self, measurement):
        if measurement:
            return measurement * 1000000 / 29 / 2

    def stop_measuring(self):
        self.stop = True

    def retrieve_state(self):
        return self.measured_distance


if __name__ == "__main__":
    sensor = UltrasonicSensor(True)
    time.sleep(100)
    sensor.stop_measuring()


