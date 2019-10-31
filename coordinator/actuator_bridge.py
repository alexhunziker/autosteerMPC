import serial
import time

from impulses import Impulses

class ActuatorBridge(object):
    def __init__(self):
        try:
            self.arduino = serial.Serial('/dev/ttyUSB0', 9600)
            self.arduino.isOpen()
            self.last_impulses = Impulses(0, 0, 0)
        except:
            print("ERROR: Connection with Arduino failed.")

    def send(self, impulses):
        if self.arduino is None:
            pritn("WARN: Arduino not available. No commands were sent")
            return
        print("INFO: Throttle", impulses.throttle, "Breaks", impulses.breaks, "Steering", impulses.steering)
        if impulses.throttle != self.last_impulses.throttle: 
            self.write_throttle(impulses.throttle)
        if impulses.breaks != self.last_impulses.breaks:
            self.write_breaks(impulses.breaks)
        if impulses.steering != self.last_impulses.steering:
            self.write_steering(impulses.steering)
        self.last_impulses = impulses

    def write_throttle(self, value):
        # the motor is very weak; therefore the range is adjusted to 200-255
        x = "0"
        if value > 0:
            x = str(int(value*55)+200)  
        control_string = "<s" + x + ">"
        print(control_string)
        self.arduino.write(control_string.encode('utf-8'))

    def write_breaks(self, value):
        # the breaks are fragile, therefore intensity is always kept below lvl 6
        if value > 0.6:
            value = 0.6
        x = str(int(value*10))
        control_string = "<b" + x + x + x + x + ">"
        print(control_string)
        self.arduino.write(control_string.encode('utf-8'))

    def write_steering(self, value):
        angle = str(int(value / 3.14 * 180) + 90)
        control_string = "<l" + angle + ">"
        print(control_string)
        self.arduino.write(control_string.encode('utf-8'))

if __name__ == "__main__":
    actuatorBridge = ActuatorBridge()
    while True:
        actuatorBridge.send(Impulses(-0.0, 0, 0))
        time.sleep(1)
        actuatorBridge.send(Impulses(0.2, 1, 0))
        time.sleep(1)
        actuatorBridge.send(Impulses(0.2, 0.5, 0.5))
        time.sleep(1)

