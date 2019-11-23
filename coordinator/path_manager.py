import sys

sys.path.append('../')

from wayplan.route_planner import RoutePlanner
from health_checker import HealthChecker 

class PathManager:
    GPS_PRECISION = 5e-4    # Empirical value

    def __init__(self, verbose=False):
        self.route_planner = RoutePlanner()
        self.health_checker = HealthChecker()
        self.verbose = verbose

    def retrieve_path(self, start, destination):
        self.route = self.route_planner.calculate(start, destination)

    def get_next(self):
        return (float(self.route[0][0]), float(self.route[0][1]))

    def potentially_update_next(self, position):
        if (abs(position["lat"] - float(self.route[0][0])) < PathManager.GPS_PRECISION and
                abs(position["lon"] - float(self.route[0][1])) < PathManager.GPS_PRECISION):
            print("INFO: Next way point loaded.")
            self.health_checker.flash_yellow()
            if len(self.route)>1:
                self.route.pop(0)
        else:
            if self.verbose:
                print("DEBUG: Distances to next are", abs(position["lat"] - float(self.route[0][0])), abs(position["lon"] - float(self.route[0][1])))


if __name__ == "__main__":
    path_manager = PathManager()
    path_manager.retrieve_path("0x80", "0x3a")
    print("current target: ", path_manager.get_next())
    path_manager.potentially_update_next({"lat": 12, "lon": 13.4, "altitude": None, "speed": None})
    print("current target: ", path_manager.get_next())
    path_manager.potentially_update_next({"lat": 50.131515, "lon": 8.66860, "altitude": None, "speed": None})
    print("current target: ", path_manager.get_next())
