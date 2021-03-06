import ctypes
import math

from impulses import Impulses
from parameters import Parameters


class MPCBridge(object):
    MPC_SO_CV_UNCONSTR = "../mpc_implementation/build/libmpc_cv_unconstr.so"
    MPC_SO_CV_OBST = "../mpc_implementation/build/libmpc_cv_obst.so"
    MPC_SO_GPS_UNCONSTR = "../mpc_implementation/build/libmpc_gps_unconstr.so"
    MPC_SO_GPS_OBST = "../mpc_implementation/build/libmpc_gps_obst.so"

    LAT_FACTOR = 111_320            # approximation
    LON_FACTOR = 40_075_000/360

    CRUISING_VELOCITY = 2.5
    TTC_CUTOFF = 4.0

    def __init__(self, silent=False, simulation_mode=False, debug=False):
        self.silent = silent
        self.simulation_mode = simulation_mode
        self.debug = debug

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
    def lat_transform(cls, x, x_ref):
        return (x-x_ref)*cls.LAT_FACTOR

    @classmethod
    def lon_transform(cls, x, x_ref, lat):
        return (x-x_ref)*cls.LON_FACTOR*math.cos(lat)

    @classmethod
    def calculate_time_to_collision(cls, distance, speed):
        # Note: Distance is expected to be in meters here
        if speed<0.1: speed = 0.1
        return distance/speed if distance>1.0 else 0

    @classmethod
    def calculate_target_speed(cls, x_target, y_target, x_current, y_current, yaw_term):
        diff_x = x_current - x_target
        diff_y = y_current - y_current
        distance_speed = max(min(pow(diff_x**2 + diff_y**2, 0.5) / 2, cls.CRUISING_VELOCITY), 1)

        yaw_speed = max(0, (cls.CRUISING_VELOCITY - 3 * yaw_term)) + 2.0
        current_appropriate_speed = min(yaw_speed, distance_speed)
        target_speed = max(0, min(current_appropriate_speed, cls.CRUISING_VELOCITY))
        print("DEBUG: Target speed", target_speed, "yaw unfiltered speed", (cls.CRUISING_VELOCITY - 3 * yaw_term) + 1.5, "distance speed", distance_speed)
        return target_speed

    @classmethod
    def calculate_target_yaw(cls, x_target, y_target, x_current, y_current, yaw):
        yaw = yaw/180*3.14
        yaw_target = math.atan2(y_target - y_current, x_target - x_current) 
        if yaw_target<0:
            yaw_target = 2*math.pi+yaw_target
        # North is 0
        print("VERBOSE: x y, current target:", x_current, y_current, x_target, y_target)
        print("VERBOSE: yaw current:", yaw , "and calculated target is:", yaw_target)

        # Adjust so we actually take shortest diff
        yaw_diff = abs(yaw - yaw_target)
        if (yaw_diff > math.pi) and yaw != 0.0:
            if (yaw_target < math.pi and yaw > math.pi):
                yaw_target = yaw_target + 2 * math.pi
            if (yaw_target > math.pi and yaw < math.pi):
                yaw_target = yaw_target - 2 * math.pi
        print("VERBOSE: sanitized target yaw:", yaw_target)
        return yaw_target 

    @classmethod
    def calculate_yaw_rate_target(cls, speed, radius):
        return speed/radius 

    def propagate_or_neutralize(self, para, speed, target):
        # We should not be confused by arbitrary directions, when driving with low speeds
        if speed > 0.8 and para!=0.0:
            return para / 180 * 3.14
        else:
            return target
        
    def request_step(self, parameters):
        time_to_collision = MPCBridge.calculate_time_to_collision(parameters.distance/100, parameters.gps["speed"])
        print("DEBUG: Time to collision is", time_to_collision)
        x_target = 0
        y_target = 0
        x_current = MPCBridge.lat_transform(parameters.gps["lat"], parameters.next_target[0])
        y_current = MPCBridge.lon_transform(parameters.gps["lon"], parameters.next_target[1], parameters.gps["lat"])
        v_target = None
        cv_available = False
        gps_available = False
        if not math.isnan(parameters.lane_curvature):
            yaw_rate_target = MPCBridge.calculate_yaw_rate_target(parameters.gps["speed"], parameters.lane_curvature)
            parameters.gps["yaw"] = self.propagate_or_neutralize(parameters.gps["yaw"], parameters.gps["speed"], 0)
            parameters.gps["yaw_rate"] = self.propagate_or_neutralize(parameters.gps["yaw_rate"], parameters.gps["speed"], yaw_rate_target)
            v_target = MPCBridge.calculate_target_speed(x_target, y_target, x_current, y_current, abs(parameters.gps["yaw_rate"]))
            cv_available = True
        elif parameters.gps["lat"] > 0:
            gps_available = True
            yaw_target = MPCBridge.calculate_target_yaw(x_target, y_target, x_current, y_current, parameters.gps["yaw"])
            parameters.gps["yaw"] = self.propagate_or_neutralize(parameters.gps["yaw"], parameters.gps["speed"], yaw_target)
            parameters.gps["yaw_rate"] = self.propagate_or_neutralize(parameters.gps["yaw_rate"], parameters.gps["speed"], 0)
            v_target = MPCBridge.calculate_target_speed(x_target, y_target, x_current, y_current, abs(yaw_target - parameters.gps["yaw"]))

        if parameters.gps["yaw"] == 0.0 or parameters.gps["yaw_rate"]>2:
            print("ERROR: Garbage values for GPS. Skipp cycle")
            return Impulses(0, 0, 0)

        if time_to_collision < MPCBridge.TTC_CUTOFF and cv_available: self.step_cv_obst(parameters, x_current, y_current, v_target, yaw_rate_target, time_to_collision)
        elif time_to_collision < MPCBridge.TTC_CUTOFF and gps_available: self.step_gps_obst(parameters, x_current, y_current, v_target, yaw_target, time_to_collision)
        elif cv_available: self.step_cv_unconstr(parameters, x_current, y_current, v_target, yaw_rate_target)
        elif gps_available: self.step_gps_unconstr(parameters, x_current, y_current, v_target, yaw_target)
        else: 
            print("WARN: Neither GPS nor CV available. Break.")
            self.last_controls = Impulses(0, 0, 1)

        if (self.emergency_break_condition(parameters)):
            return self.apply_emergency_break(self.last_controls)
        return self.last_controls

    def step_gps_unconstr(self, parameters, x_current, y_current, v_target, yaw_target):
        if self.debug:
            print("GPS Model w/o obstacle will be applied.")
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
            mpc_controls_receiver,
            mpc_state_receiver
        )
        planned_impulses = list(mpc_controls_receiver)
        self.last_controls = Impulses(planned_impulses[0], planned_impulses[1], planned_impulses[2])

    def step_gps_obst(self, parameters, x_current, y_current, v_target, yaw_target, time_to_collision):
        if self.debug:
            print("DEBUG: GPS Model with obstacle will be applied.")
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
            ctypes.c_double(time_to_collision),
            mpc_controls_receiver,
            mpc_state_receiver
        )
        planned_impulses = list(mpc_controls_receiver)
        self.last_controls = Impulses(planned_impulses[0], planned_impulses[1], planned_impulses[2])

    def step_cv_unconstr(self, parameters, x_current, y_current, v_target, yaw_rate_target):
        if self.debug:
            print("DEBUG: CV Model w/o obstacle will be applied.")
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
            mpc_controls_receiver,
            mpc_state_receiver
        )
        planned_impulses = list(mpc_controls_receiver)
        self.last_controls = Impulses(planned_impulses[0], planned_impulses[2], planned_impulses[3])

    def step_cv_obst(self, parameters, x_current, y_current, v_target, yaw_rate_target, time_to_collision):
        if self.debug:
            print("DEBUG: CV Model with obstacle will be applied.")
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
            mpc_controls_receiver,
            mpc_state_receiver
        )
        planned_impulses = list(mpc_controls_receiver)
        self.last_controls = Impulses(planned_impulses[0], planned_impulses[2], planned_impulses[3])

    def emergency_break_condition(self, parameters):
        return (parameters.distance < 400) and (parameters.distance < 2 * parameters.gps["speed"]) or parameters.distance<100

    def apply_emergency_break(self, impulses):
        impulses.breaks = 1.0
        impulses.throttle = 0.0
        if (not self.silent):
            print("TNFO: Break applied, time to impact lesss than 2s or distance <1m")
        return impulses


if __name__ == "__main__":
    parameters = Parameters()
    parameters.next_target = [10, -10]
    parameters.gps["lat"] = 0
    parameters.gps["lon"] = 0
    parameters.gps["speed"] = 3.9
    parameters.gps["yaw"] = 0.2
    parameters.gps["yaw_rate"] = 0.2
    MPCBridge(debug=True).request_step(parameters)
