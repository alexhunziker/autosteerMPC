import ctypes

from impulses import Impulses
from parameters import Parameters


class MPCBridge(object):
    MPC_SHARED_OBJECT_LOCATION = "../mpc_dlib/build/libmpc_controller.so"

    MPC_SO_CV_UNCONSTR = "../mpc_implementation/build/libmpc_cv_unconstr.so"
    MPC_SO_CV_UNCONSTR = "../mpc_implementation/build/libmpc_cv_obst.so"
    MPC_SO_CV_UNCONSTR = "../mpc_implementation/build/libmpc_cv_unconstr.so"
    MPC_SO_CV_UNCONSTR = "../mpc_implementation/build/libmpc_gps_obst.so"

    LAT_FACTOR = 70_000
    LON_FACTOR = 111_000
    LAT_OFFSET = 50.128798
    LON_OFFSET = 8.667482

    def __init__(self, silent=False):
        self.silent = silent

        self.mpc_controller = ctypes.CDLL(MPCBridge.MPC_SHARED_OBJECT_LOCATION)
        self.mpc_controller.initialize_mpc_objects()
        self.last_controls = Impulses(0, 0, 0)

    @classmethod
    def lat_transform(cls, x):
        return (x-cls.LAT_OFFSET)*cls.LAT_FACTOR

    @classmethod
    def lon_transform(cls, x):
        return (x-cls.LON_OFFSET)*cls.LON_FACTOR

    def request_step(self, parameters):
        x_target = MPCBridge.lat_transform(parameters.next_target[0])
        y_target = MPCBridge.lon_transform(parameters.next_target[1])
        x_current = MPCBridge.lat_transform(parameters.gps["lat"])
        y_current = MPCBridge.lon_transform(parameters.gps["lon"])
        mpc_controls_receiver = (ctypes.c_double * 3)()
        self.mpc_controller.predict(ctypes.c_double(x_target),
                                    ctypes.c_double(y_target),
                                    ctypes.c_double(self.last_controls.steering),
                                    ctypes.c_double(self.last_controls.throttle),
                                    ctypes.c_double(self.last_controls.breaks),
                                    ctypes.c_double(x_current), ctypes.c_double(y_current),
                                    ctypes.c_double(parameters.gps["speed"]), mpc_controls_receiver
                                    )
        planned_impulses = list(mpc_controls_receiver)
        self.last_controls = Impulses(planned_impulses[0], planned_impulses[1], planned_impulses[2])
        if (self.emergency_break_condition(parameters)):
            return self.apply_emergency_brak(self.last_controls)
        return self.last_controls

    def emergency_break_condition(self, parameters):
        return (parameters.distance < 400) and (parameters.distance < 2 * parameters.speed)

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
