import serial
import time
import numpy as np
import os
import sys

from impulses import Impulses

class ActuatorBridge(object):

    def __init__(self, mock=False):
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

    def send(self, impulses):
        if self.busy:
            print("INFO: Commands skipped, actuator bridge is busy")
            return
        self.busy = True
        if self.arduino is None:
            print("WARN: Arduino not available. No commands were sent")
            return
        print("INFO: Throttle", impulses.throttle, "Breaks", impulses.breaks, "Steering", impulses.steering)
        # Let's try to send an impuls all the time... Since some writes are ignored by the Arduino... Viel hilft vieleicht viel
        if impulses.throttle != self.last_impulses.throttle or impulses.breaks != self.last_impulses.breaks or impulses.steering != self.last_impulses.steering or True: 
            # Steering angle in degrees, plus offset
            if abs(impulses.steering-self.last_impulses.steering)>0.05:
                impulses.steering = 0.05*np.sign(impulses.steering-self.last_impulses.steering)
                print("INFO: Smoothened steering")
            steering_angle = str(int(impulses.steering  / 3.14 * 180) + 50)
            if impulses.throttle > 0.4:
                impulses.throttle = 0.4
                print("INFO: Throttle adjusted")
            throttle = str(int(impulses.throttle*10));
            # the breaks are fragile, therefore intensity is always kept below lvl 6
            if impulses.breaks > 0.6:
                impulses.breaks = 0.6
            break_intensity = str(int(impulses.breaks*10))
            control_string = "<" + steering_angle + throttle + break_intensity + ">"
            self.arduino.write(control_string.encode('utf-8'))
            time.sleep(0.1)                                 # Give Arduino a bit time
        self.last_impulses = impulses
        self.busy = False

    def mock_send(self, impulses):
        print("INFO: Throttle", impulses.throttle, "Breaks", impulses.breaks, "Steering", impulses.steering)
        print("INFO: Mock actuator bridge. SEND NOTHING")

if __name__ == "__main__":
    try:
        actuatorBridge = ActuatorBridge()
        #for i in range(-30, 30, 5):
        #    print("Test steering; ", i/100)
        #    actuatorBridge.send(Impulses(i/100, 0, 0))
        #    time.sleep(0.2)
        #for i in range(0, 5, 1):
        #    print("Test throttle; ", i/10)
        #    actuatorBridge.send(Impulses(0, i/10, 0))
        #    time.sleep(2)
        actuatorBridge.send(Impulses(0.0, 0.0, 0.0))
        time.sleep(2)
        #actuatorBridge.send(Impulses(0, 0.3, 0.0))
        #time.sleep(2)
        while True:
            actuatorBridge.send(Impulses(0.1, 0.3, 0.0))
            actuatorBridge.send(Impulses(0.0, 0.0, 0.0))
            print("set")
        actuatorBridge.send(Impulses(0.0, 0.0, 0.0))
        print("DONE")
    except KeyboardInterrupt:
        print("INFO: Shutting down")
        ActuatorBridge(mock=False).send(Impulses(0, 0, 0))
        time.sleep(1)
        ActuatorBridge(mock=False).send(Impulses(0, 0, 0))
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)


