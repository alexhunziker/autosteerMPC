import threading
import time

import RPi.GPIO as GPIO


class HealthChecker(object):
    SLACK_ULTRASONIC = 3

    GPIO_GREEN = 21
    GPIO_RED = 20

    def __init__(self):
        self.ultrasonic_fails = 0
        self.ultrasonic_last = 0

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(HealthChecker.GPIO_GREEN, GPIO.OUT)
        GPIO.setup(HealthChecker.GPIO_RED, GPIO.OUT)

    def ready(self):
        GPIO.output(HealthChecker.GPIO_GREEN, GPIO.HIGH)

    def error(self):
        GPIO.output(HealthChecker.GPIO_RED, GPIO.HIGH)

    def check(self, parameters):
        fail = False
        if self.ultrasonic_last == parameters.ultrasonic or parameters.ultrasonic is None:
            self.ultrasonic_fails += 1

        if self.ultrasonic_fails > HealthChecker.SLACK_ULTRASONIC:
            fail = True

        flash = threading.Thread(target=self.flash, parameters=fail)
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
