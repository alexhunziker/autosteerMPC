from wayplan import RoutePlanner

class PathManager(object):
    def __init__(self):
        self.route_planner = RoutePlanner()

    def retrieve_path(self, start, destination):
        self.route = self.route_planner.calculate(start, destination)

    def get_next(self):
        return self.route[0]

    def potentially_update_next(self, position):
        raise NotImplementedError
