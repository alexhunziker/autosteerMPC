import ctypes

from impulses import Impulses
from parameters import Parameters


class MPCBridge(object):
    MPC_SHARED_OBJECT_LOCATION = "../mpc_dlib/build/libmpc_controller.so"

    def __init__(self, silent=False):
        self.silent = silent

        self.mpc_controller = ctypes.CDLL(MPCBridge.MPC_SHARED_OBJECT_LOCATION)
        self.mpc_controller.initialize_mpc_objects()
        self.last_controls = Impulses(0, 0, 0)

    def request_step(self, parameters):
        mpc_controls_receiver = (ctypes.c_double * 3)()
        self.mpc_controller.predict(ctypes.c_double(10), ctypes.c_double(10),
                                    ctypes.c_double(self.last_controls.steering),
                                    ctypes.c_double(self.last_controls.throttle),
                                    ctypes.c_double(self.last_controls.breaks),
                                    ctypes.c_double(parameters.gps["lat"]), ctypes.c_double(parameters.gps["lon"]),
                                    ctypes.c_double(parameters.gps["speed"]), mpc_controls_receiver
                                    )
        planned_impulses = list(mpc_controls_receiver)
        self.last_controls = Impulses(planned_impulses[0], planned_impulses[1], planned_impulses[2])
        if (self.emergency_break_condition(parameters)):
            return self.apply_emergency_brak(self.last_controls)
        return self.last_controls

    def emergency_break_condition(self, parameters):
        return (parameters.ultrasonic < 400) and (parameters.ultrasonic < 2 * parameters.speed)

    def apply_emergency_break(self, impulses):
        impulses.breaks = 1.0
        impulses.throttle = 0.0
        if (not self.silent):
            print("TNFO: EMERGENCY BREAK APPLIED")
        return impulses


if __name__ == "__main__":
    parameters = Parameters()
    parameters.gps["lat"] = 0
    parameters.gps["lon"] = 0
    parameters.gps["speed"] = 3.9
    MPCBridge().request_step(parameters)
