import serial
import time

class ActuatorBridge(object):
    def __init__(self, mock=False):
        self.connection = serial.Serial('/dev/ttyUSB1', 115200)
        self.connection.isOpen()
        self.last_impulses = None
        if mock:
            self.send = self.mock_send

    def send(self, impulses):
        print("INFO: Throttle", impulses.throttle, "Breaks", impulses.breaks, "Steering", impulses.steering)
        if impulses.throttle != self.last_impulses.throttle: 
            write_throttle(impulses.throttle)
        if impulses.breaks != self.last_impulses.breaks:
            write_breaks(impulses.breaks)
        if impulses.steering != self.last_impulses.steering:
            write_steering(impulses.steering)
        self.last_impulses = impulses

    def write_throttle(self, value):
        #control_string = "<s" + "{0:0=3d}".format(int(value*255)) + ">"
        control_string = "<s" + str(int(value*255)) + ">"
        self.connection.write(control_string.encode('utf-8'))

    def write_breaks(self, value):
        x = str(int(self*10))
        control_string = "<b" + x + x + x + x + ">"
        self.connection.write(control_string.encode('utf-8'))

    def write_steering(self, value):
        angle = str(int(value / 3.14 * 180) + 90)
        control_string = "<l" + angle + ">"
        self.connection.write(control_string.encode('utf-8'))

    def mock_send(self, impulses):
        print("INFO: Throttle", impulses.throttle, "Breaks", impulses.breaks, "Steering", impulses.steering)
        print("INFO: Mock actuator bridge. SEND NOTHING")

if __name__ == "__main__":
    actuatorBridge = ActuatorBridge()
    while True:
        actuatorBridge.send(Impulses(0, 0, 0))
        time.sleep(1)
        actuatorBridge.send(Impulses(0, 1, 0))
        time.sleep(1)

