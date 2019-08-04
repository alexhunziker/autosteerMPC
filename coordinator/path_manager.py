import sys

sys.path.append('../')

from wayplan.route_planner import RoutePlanner


class PathManager:
    GPS_PRECISION = 2.5

    def __init__(self):
        self.route_planner = RoutePlanner()

    def retrieve_path(self, start, destination):
        self.route = self.route_planner.calculate(start, destination)

    def get_next(self):
        return self.route[0]

    def potentially_update_next(self, position):
        if (abs(position["lat"] - float(self.route[0][0])) < PathManager.GPS_PRECISION and
                abs(position["lon"] - float(self.route[0][1])) < PathManager.GPS_PRECISION):
            print("INFO: Next way point loaded.")
            self.route.pop(0)
