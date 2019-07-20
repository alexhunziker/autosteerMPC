import RPi.GPIO as GPIO
import time

class UltrasonicSensor(object):
    GPIO_PIN = 6
    TIMEOUT_1 = 1000
    TIMEOUT_2 = 10000

    usleep = lambda x: time.sleep(x / 1_000_000.0)

    def __init__(self):
        GPIO.setmode(GPIO.BCM)

    def measure(self):
        self.trigger_signal()
        measurement = self.receive_signal()
        return self.calculate_distance_cm(measurement)

    def trigger_signal(self):
        GPIO.setup(UltrasonicSensor.GPIO_PIN, GPIO.OUT)
        GPIO.output(UltrasonicSensor.GPIO_PIN, GPIO.LOW)
        UltrasonicSensor.usleep(2)
        GPIO.output(UltrasonicSensor.GPIO_PIN, GPIO.HIGH)
        UltrasonicSensor.usleep(10)
        GPIO.output(UltrasonicSensor.GPIO_PIN, GPIO.LOW)

    def receive_signal(self):
        # Todo: maybe we loose time here by calling the function
        t_0 = time.time()
        GPIO.setup(UltrasonicSensor.GPIO_PIN, GPIO.IN)
        count = 0
        while count < UltrasonicSensor.TIMEOUT_1:
            if GPIO.input(UltrasonicSensor.GPIO_PIN):
                break
            count += 1
        if count >= UltrasonicSensor.TIMEOUT_1:
            print("M1 failed")
            return None
        t_1 = time.time()
        while count < UltrasonicSensor.TIMEOUT_2:
            if not GPIO.input(UltrasonicSensor.GPIO_PIN):
                break
            print("true")
            count += 1
        if count >= UltrasonicSensor.TIMEOUT_2:
            print("M2 failed")
            return None
        t_2 = time.time()
        dt = int((t_1 - t_0) * 1000000)
        if dt > 530:
            return None
        return t_2 - t_1

    def calculate_distance_cm(self, measurement):
        if measurement:
            return measurement * 1000000 / 29 / 2

if __name__ == "__main__":
    distance = UltrasonicSensor().measure()
    print("Distance is", distance, "cm")
