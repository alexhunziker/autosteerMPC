import ctypes
import math

from impulses import Impulses
from parameters import Parameters


class MPCBridge(object):
    MPC_SO_CV_UNCONSTR = "../mpc_implementation/build/libmpc_cv_unconstr.so"
    MPC_SO_CV_OBST = "../mpc_implementation/build/libmpc_cv_obst.so"
    MPC_SO_GPS_UNCONSTR = "../mpc_implementation/build/libmpc_cv_unconstr.so"
    MPC_SO_GPS_OBST = "../mpc_implementation/build/libmpc_gps_obst.so"

    LAT_FACTOR = 70_000 # TODO: Better Approximation 
    LON_FACTOR = 111_000
    LAT_OFFSET = 50.128798
    LON_OFFSET = 8.667482

    CRUISING_VELOCITY = 4.0

    def __init__(self, silent=False, simulation_mode=False):
        self.silent = silent
        self.simulation_mode = simulation_mode

        self.mpc_cv_unconstr = ctypes.CDLL(MPCBridge.MPC_SO_CV_UNCONSTR)
        self.mpc_cv_unconstr.initialize_mpc_objects()
        self.mpc_cv_obst = ctypes.CDLL(MPCBridge.MPC_SO_CV_OBST)
        self.mpc_cv_obst.initialize_mpc_objects()
        self.mpc_gps_unconstr = ctypes.CDLL(MPCBridge.MPC_SO_GPS_UNCONSTR)
        self.mpc_gps_unconstr.initialize_mpc_objects()
        self.mpc_gps_obst = ctypes.CDLL(MPCBridge.MPC_SO_GPS_OBST)
        self.mpc_gps_obst.initialize_mpc_objects()

        self.last_controls = Impulses(0, 0, 0)
        self.schwimm = 0.0

    @classmethod
    def lat_transform(cls, x):
        return (x-cls.LAT_OFFSET)*cls.LAT_FACTOR

    @classmethod
    def lon_transform(cls, x):
        return (x-cls.LON_OFFSET)*cls.LON_FACTOR

    @classmethod
    def calculate_time_to_collision(cls, distance, speed):
        return distance/speed if distance>1.0 else 0

    @classmethod
    def calculate_target_speed(cls, x_target, y_target, x_current, y_current, yaw_term):
        diff_x = x_current - x_target
        diff_y = y_current - y_current
        distance_speed = pow(diff_x**2 + diff_y**2, 0.5) / 2

        yaw_speed = (cls.CRUISING_VELOCITY - 3 * yaw_term) + 1.5
        current_appropriate_speed = min(yaw_speed, distance_speed)
        target_speed = min(current_appropriate_speed, cls.CRUISING_VELOCITY)
        return target_speed

    @classmethod
    def calculate_target_yaw(cls, x_target, y_target, x_current, y_current, yaw):
        yaw_target = math.atan2(y_target - y_current, x_target - x_current)

        # Adjust so we actually take shortest diff
        yaw_diff = abs(yaw - yaw_target)
        if (yaw_diff > math.pi):
            if (yaw_target < 0 and yaw > 0):
                yaw_target = yaw_target + 2 * math.pi
            if (yaw_target > 0 and yaw < 0):
                yaw_target = yaw_target - 2 * math.pi
        return yaw_target 

    @classmethod
    def calculate_yaw_rate_target(cls, speed, radius):
        return speed/radius # TODO: Double check this...

    def request_step(self, parameters):
        time_to_collision = MPCBridge.calculate_time_to_collision(parameters.distance, parameters.speed)
        x_target = MPCBridge.lat_transform(parameters.next_target[0])
        y_target = MPCBridge.lon_transform(parameters.next_target[1])
        x_current = MPCBridge.lat_transform(parameters.gps["lat"])
        y_current = MPCBridge.lon_transform(parameters.gps["lon"])
        v_target = None
        cv_available = False
        if parameters.lane_curvature > 0:
            yaw_rate_target = MPCBridge.calculate_yaw_rate_target(parameters.gps["speed"], parameters.lane_curvature)
            cv_available = True
        else:
            yaw_target = MPCBridge.calculate_target_yaw(x_target, y_target, x_current, y_current, parameters.gps["yaw"])
            v_target = MPCBridge.calculate_target_speed(x_target, y_target, x_current, y_current, abs(yaw_target - parameters.gps["yaw"]))

        impulses = None
        if time_to_collision < 3 and cv_available: impulses = self.step_cv_obst()
        elif time_to_collision < 3: impulses = self.step_gps_obst(parameters, x_current, y_current, v_target, yaw_target, time_to_collision)
        elif cv_available: impulses = self.step_cv_unconstr(parameters, x_current, y_current, v_target, yaw_rate_target)
        else: impulses = self.step_gps_unconstr(parameters, x_current, y_current, v_target, yaw_target)

        planned_impulses = list(impulses)
        self.last_controls = Impulses(planned_impulses[0], planned_impulses[1], planned_impulses[2])
        if (self.emergency_break_condition(parameters)):
            return self.apply_emergency_break(self.last_controls)
        return self.last_controls

    def step_gps_unconstr(self, parameters, x_current, y_current, v_target, yaw_target):
        mpc_controls_receiver = (ctypes.c_double * 3)()
        mpc_state_receiver = (ctypes.c_double * 4)()
        self.mpc_gps_unconstr.predict(
            ctypes.c_double(v_target),
            ctypes.c_double(x_current), 
            ctypes.c_double(y_current),
            ctypes.c_double(parameters.gps["speed"]),
            ctypes.c_double(parameters.gps["yaw"]),
            ctypes.c_double(parameters.gps["yaw_rate"]),
            ctypes.c_double(self.schwimm),
            ctypes.c_double(yaw_target),
            mpc_state_receiver,
            mpc_controls_receiver
        )
        return mpc_controls_receiver

    def step_gps_obst(self, parameters, x_current, y_current, v_target, yaw_target, time_to_collision):
        mpc_controls_receiver = (ctypes.c_double * 3)()
        mpc_state_receiver = (ctypes.c_double * 5)()
        self.mpc_gps_obst.predict(
            ctypes.c_double(v_target),
            ctypes.c_double(x_current), 
            ctypes.c_double(y_current),
            ctypes.c_double(parameters.gps["speed"]),
            ctypes.c_double(parameters.gps["yaw"]),
            ctypes.c_double(parameters.gps["yaw_rate"]),
            ctypes.c_double(self.schwimm),
            ctypes.c_double(yaw_target),
            mpc_state_receiver,
            mpc_controls_receiver
        )
        return mpc_controls_receiver

    def step_cv_unconstr(self, parameters, x_current, y_current, v_target, yaw_rate_target):
        mpc_controls_receiver = (ctypes.c_double * 4)()
        mpc_state_receiver = (ctypes.c_double * 6)()
        self.mpc_cv_unconstr.predict(
            ctypes.c_double(v_target),
            ctypes.c_double(x_current), 
            ctypes.c_double(y_current),
            ctypes.c_double(parameters.gps["speed"]),
            ctypes.c_double(parameters.lateral_offset),
            ctypes.c_double(parameters.gps["yaw"]),
            ctypes.c_double(parameters.gps["yaw_rate"]),
            ctypes.c_double(self.schwimm),
            ctypes.c_double(yaw_rate_target),
            mpc_state_receiver,
            mpc_controls_receiver
        )
        return mpc_controls_receiver

    def step_cv_obst(self, parameters, x_current, y_current, v_target, yaw_rate_target, time_to_collision):
        mpc_controls_receiver = (ctypes.c_double * 4)()
        mpc_state_receiver = (ctypes.c_double * 7)()
        self.mpc_cv_obst.predict(
            ctypes.c_double(v_target),
            ctypes.c_double(x_current), 
            ctypes.c_double(y_current),
            ctypes.c_double(parameters.gps["speed"]),
            ctypes.c_double(parameters.lateral_offset),
            ctypes.c_double(parameters.gps["yaw"]),
            ctypes.c_double(parameters.gps["yaw_rate"]),
            ctypes.c_double(self.schwimm),
            ctypes.c_double(yaw_rate_target),
            ctypes.c_double(time_to_collision),
            mpc_state_receiver,
            mpc_controls_receiver
        )
        return mpc_controls_receiver

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
