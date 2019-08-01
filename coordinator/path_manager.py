from wayplan import RoutePlanner

class PathManager(object):
    GPS_PRECISION = 2.5

    def __init__(self):
        self.route_planner = RoutePlanner()

    def retrieve_path(self, start, destination):
        self.route = self.route_planner.calculate(start, destination)

    def get_next(self):
        return self.route[0]

    def potentially_update_next(self, position):
        if (abs(position.lat - self.route[0][0]) < PathManager.GPS_PRECISION and
                abs(position.lat - self.route[0][1]) < PathManager.GPS_PRECISION):
            self.route.pop(0)
