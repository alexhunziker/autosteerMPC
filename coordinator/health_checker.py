import threading
import time

import RPi.GPIO as GPIO


class HealthChecker(object):
    MAX_SLACK_ULTRASONIC = 0.2
    MAX_SLACK_GPS = 0.5

    GPIO_GREEN = 21
    GPIO_RED = 20

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(HealthChecker.GPIO_GREEN, GPIO.OUT)
        GPIO.setup(HealthChecker.GPIO_RED, GPIO.OUT)

    def ready(self):
        GPIO.output(HealthChecker.GPIO_GREEN, GPIO.HIGH)

    def error(self):
        GPIO.output(HealthChecker.GPIO_RED, GPIO.HIGH)

    def computing(self):
        double_flash = threading.Thread(target=self.double_flash)
        double_flash.start()

    def double_flash(self):
        for _ in range(30):
            GPIO.output(HealthChecker.GPIO_RED, GPIO.HIGH)
            GPIO.output(HealthChecker.GPIO_GREEN, GPIO.HIGH)
            time.sleep(0.1)
            GPIO.output(HealthChecker.GPIO_RED, GPIO.LOW)
            GPIO.output(HealthChecker.GPIO_GREEN, GPIO.LOW)
            time.sleep(0.1)
            break

    def check(self, parameters):
        fail = False
        now = time.time()

        if (now - parameters.ultrasonic_timestamp) > HealthChecker.MAX_SLACK_ULTRASONIC:
            print("WARNING: Ultrasonic data out of date, last update", parameters.ultrasonic_timestamp)
            fail = True

        if (now - parameters.gps_timestamp) > HealthChecker.MAX_SLACK_GPS:
            print("WARNING: GPS data out of date, last update", parameters.gps_timestamp)
            fail = True

        flash = threading.Thread(target=self.flash, args=[fail])
        flash.start()

    def flash(self, fail):
        if fail:
            GPIO.output(HealthChecker.GPIO_RED, GPIO.HIGH)
            time.sleep(0.05)
            GPIO.output(HealthChecker.GPIO_RED, GPIO.LOW)
        else:
            GPIO.output(HealthChecker.GPIO_GREEN, GPIO.HIGH)
            time.sleep(0.05)
            GPIO.output(HealthChecker.GPIO_GREEN, GPIO.LOW)
