import threading
import time

import RPi.GPIO as GPIO


class HealthChecker(object):
    MAX_SLACK_DISTANCE = 0.2
    MAX_SLACK_GPS = 0.5
    MAX_SLACK_CV = 1.0

    GPIO_GREEN = 21
    GPIO_RED = 20
    GPIO_BLLUE = 26
    GPIO_YELLOW = 16


    def __init__(self, verbose = False):
        self.verbose = verbose
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(HealthChecker.GPIO_GREEN, GPIO.OUT)
        GPIO.setup(HealthChecker.GPIO_RED, GPIO.OUT)
        GPIO.setup(HealthChecker.GPIO_BLLUE, GPIO.OUT)
        GPIO.setup(HealthChecker.GPIO_YELLOW, GPIO.OUT)

    def ready(self):
        GPIO.output(HealthChecker.GPIO_YELLOW, GPIO.HIGH)

    def error(self):
        GPIO.output(HealthChecker.GPIO_RED, GPIO.HIGH)

    def computing(self):
        double_flash = threading.Thread(target=self.double_flash)
        double_flash.start()

    def double_flash(self):
        GPIO.output(HealthChecker.GPIO_RED, GPIO.HIGH)
        GPIO.output(HealthChecker.GPIO_GREEN, GPIO.HIGH)
        GPIO.output(HealthChecker.GPIO_BLLUE, GPIO.HIGH)
        GPIO.output(HealthChecker.GPIO_YELLOW, GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(HealthChecker.GPIO_RED, GPIO.LOW)
        GPIO.output(HealthChecker.GPIO_GREEN, GPIO.LOW)
        GPIO.output(HealthChecker.GPIO_BLLUE, GPIO.LOW)
        GPIO.output(HealthChecker.GPIO_YELLOW, GPIO.LOW)
        time.sleep(0.2)

    def check(self, parameters):
        fail = False
        now = time.time()

        if (now - parameters.distance_timestamp) > HealthChecker.MAX_SLACK_DISTANCE:
            print("WARNING: Distance data out of date, age", now-parameters.distance_timestamp)
            fail = True

        if (now - parameters.gps_timestamp) > HealthChecker.MAX_SLACK_GPS:
            print("WARNING: GPS data out of date, age", now-parameters.gps_timestamp)
            fail = True

        if (now - parameters.cv_timestamp) < HealthChecker.MAX_SLACK_CV:
            GPIO.output(HealthChecker.GPIO_BLLUE, GPIO.HIGH)
            if self.verbose: print("INFO: CV data available, age", now-parameters.cv_timestamp)
        else:
            GPIO.output(HealthChecker.GPIO_BLLUE, GPIO.LOW)
            if self.verbose: print("INFO: CV data out of date, age", now-parameters.cv_timestamp)

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

if __name__ == "__main__":
    healthChecker = HealthChecker(verbose=True)
    healthChecker.double_flash()
    time.sleep(1)
    healthChecker.ready()
    time.sleep(1)
    healthChecker.flash(True)
    time.sleep(1)
    healthChecker.flash(False)
