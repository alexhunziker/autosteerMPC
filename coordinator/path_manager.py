import sys

sys.path.append('../')

from wayplan.route_planner import RoutePlanner


class PathManager:
    # TODO: This needs to be scaled down.
    GPS_PRECISION = 2.5

    def __init__(self):
        self.route_planner = RoutePlanner()

    def retrieve_path(self, start, destination):
        self.route = self.route_planner.calculate(start, destination)

    def get_next(self):
        return (float(self.route[0][0]), float(self.route[0][1]))

    def potentially_update_next(self, position):
        if (abs(position["lat"] - float(self.route[0][0])) < PathManager.GPS_PRECISION and
                abs(position["lon"] - float(self.route[0][1])) < PathManager.GPS_PRECISION):
            print("INFO: Next way point loaded.")
            self.route.pop(0)


if __name__ == "__main__":
    path_manager = PathManager()
    path_manager.retrieve_path("0x80", "0x3a")
    print("current target: ", path_manager.get_next())
    path_manager.potentially_update_next({"lat": 12, "lon": 13.4, "altitude": None, "speed": None})
    print("current target: ", path_manager.get_next())
    path_manager.potentially_update_next({"lat": 50.131515, "lon": 8.66860, "altitude": None, "speed": None})
    print("current target: ", path_manager.get_next())
