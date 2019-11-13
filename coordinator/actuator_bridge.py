import serial
import time

from impulses import Impulses

class ActuatorBridge(object):

    def __init__(self, mock=False):
        if mock:
            self.send = self.mock_send
            return
        try:
            self.arduino = serial.Serial('/dev/ttyUSB0', 9600)
            self.arduino.isOpen()
            self.last_impulses = Impulses(0, 0, 0)
        except:
            print("ERROR: Connection with Arduino failed.")
        self.busy = False # Because the vehicle is slow with processing, otherwise signals get lostS

    def send(self, impulses):
        if self.busy:
            print("INFO: Commands skipped, actuator bridge is busy")
            return
        self.busy = True
        if self.arduino is None:
            print("WARN: Arduino not available. No commands were sent")
            return
        print("INFO: Throttle", impulses.throttle, "Breaks", impulses.breaks, "Steering", impulses.steering)
        if impulses.throttle != self.last_impulses.throttle: 
            self.write_throttle(impulses.throttle)
            time.sleep(0.5)
        if impulses.breaks != self.last_impulses.breaks:
            self.write_breaks(impulses.breaks)
            time.sleep(0.5)
        if impulses.steering != self.last_impulses.steering:
            self.write_steering(impulses.steering)
            time.sleep(0.5)
        self.last_impulses = impulses
        self.busy = False

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

    def mock_send(self, impulses):
        print("INFO: Throttle", impulses.throttle, "Breaks", impulses.breaks, "Steering", impulses.steering)
        print("INFO: Mock actuator bridge. SEND NOTHING")

if __name__ == "__main__":
    actuatorBridge = ActuatorBridge()
    for i in range(-4, 5, 1):
        print("Test steering; ", i/10)
        actuatorBridge.send(Impulses(i/10, 0, 0))
        time.sleep(1)
    for i in range(0, 10, 1):
        print("Test throttle; ", i/10)
        actuatorBridge.send(Impulses(0, i/10, 0))
        time.sleep(1)
    while True:
        #actuatorBridge.send(Impulses(-0.0, 0, 0))
        #time.sleep(1)
        actuatorBridge.send(Impulses(0, 1, 0))
        time.sleep(10)
        #actuatorBridge.send(Impulses(0.2, 0.5, 0.5))
        #time.sleep(1)

