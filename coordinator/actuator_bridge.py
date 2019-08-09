class ActuatorBridge(object):
    def send(self, impulses):
        print("DEBUG: Throttle", impulses.throttle, "Breaks", impulses.breaks, "Steering", impulses.steering)
