from impulses import Impulses

class MPCBridge(object):
    def __init__(self, silent=False):
        self.silent = silent
        return None
        # Initialize all MPC Models here

    def request_step(self, parameters):
        # here goes the regular mpc stuff
        impulses = Impulses(0, 0, 0)
        if (self.emergency_break_condition(parameters)):
            return self.apply_emergency_brak(impulses)
        return impulses

    def emergency_break_condition(self, parameters):
        return (parameters.ultrasonic < 400) and (parameters.ultrasonic < 2 * parameters.speed)

    def apply_emergency_break(self, impulses):
        impulses.breaks = 1.0
        impulses.throttle = 0.0
        if (not self.silent):
            print("TNFO: EMERGENCY BREAK APPLIED")
        return impulses
