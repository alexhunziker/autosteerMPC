import serial
import time
import numpy as np
import os
import sys

from impulses import Impulses

class ActuatorBridge(object):

    def __init__(self, mock=False, smoothing=True):
        if mock:
            self.send = self.mock_send
            return
        try:
            self.arduino = serial.Serial('/dev/arduino', 9600)
            self.arduino.isOpen()
            self.last_impulses = Impulses(0, 0, 0)
        except:
            print("ERROR: Connection with Arduino failed.")
        self.busy = False # Because the vehicle is slow with processing, otherwise signals get lost
        self.smoothing = smoothing

    def send(self, impulses):
        if self.busy:
            print("INFO: Commands skipped, actuator bridge is busy")
            return
        self.busy = True
        if self.arduino is None:
            print("WARN: Arduino not available. No commands were sent")
            return
        print("INFO: Throttle", impulses.throttle, "Breaks", impulses.breaks, "Steering", impulses.steering)

        # Steering angle in degrees, inverted, plus offset
        if self.smoothing and abs(impulses.steering-self.last_impulses.steering)>0.05:
            print(np.sign(impulses.steering-self.last_impulses.steering))
            tmp_max_allowed = 0.05*np.sign(impulses.steering-self.last_impulses.steering)+self.last_impulses.steering
            if abs(tmp_max_allowed)<abs(impulses.steering):
                impulses.steering = tmp_max_allowed
                print("INFO: Smoothened steering to", impulses.steering)
        steering_angle = str(int(-impulses.steering / 3.14 * 180) + 50)
        # The engine is powerful, limit acceleration and speed

        if impulses.throttle > 0.5:
            impulses.throttle = 0.5
            print("INFO: Throttle adjusted")
        throttle = str(int(impulses.throttle*10))

        # the breaks are fragile, therefore intensity is always kept below lvl 6
        if impulses.breaks > 0.6:
            impulses.breaks = 0.6
        break_intensity = str(int(impulses.breaks*10))

        control_string = "<" + steering_angle + throttle + break_intensity + ">"
        self.arduino.write(control_string.encode('utf-8'))
        print("DEBUG: Control string sent", control_string)
        time.sleep(0.1)                                 # Give Arduino a bit time
        self.last_impulses = impulses
        self.busy = False

    def mock_send(self, impulses):
        print("INFO: Throttle", impulses.throttle, "Breaks", impulses.breaks, "Steering", impulses.steering)
        print("INFO: Mock actuator bridge. SEND NOTHING")

if __name__ == "__main__":
    try:
        actuatorBridge = ActuatorBridge(smoothing=False)
        time.sleep(3)

        print("Test Steering...")
        actuatorBridge.send(Impulses(-0.3, 0.0, 0.0)) # left
        time.sleep(2)
        actuatorBridge.send(Impulses(0.3, 0.0, 0.0)) # right
        time.sleep(2)
        print("Test Acceleration...")
        actuatorBridge.send(Impulses(0, 1, 0.0))
        time.sleep(0.5)
        print("Test Brakes...")
        actuatorBridge.send(Impulses(0, 0, 1))
        time.sleep(0.5)
        print("done.")
        actuatorBridge.send(Impulses(0, 0, 0))

        print("Test loop...")
        while True:
            actuatorBridge.send(Impulses(-0.2, 1.0, 0.0))
            time.sleep(2)
            actuatorBridge.send(Impulses(0.2, 0.0, 0.0))
            time.sleep(2)
        time.sleep(2)
        actuatorBridge.send(Impulses(0, 0.0, 0.0))
        print("done.")
    except KeyboardInterrupt:
        print("INFO: Shutting down")
        ActuatorBridge(mock=False).send(Impulses(0, 0, 0))
        time.sleep(1)
        ActuatorBridge(mock=False).send(Impulses(0, 0, 0))
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)


