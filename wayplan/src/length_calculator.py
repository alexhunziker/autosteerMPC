from math import cos, sqrt, radians

class LengthCalculator(object):
    """Distance estimation based on Equirectangular projection"""

    RADIUS_EARTH = 6_371

    def estimate(self, p1, p2):
        lat_1 = radians(float(p1[0]))
        lon_1 = radians(float(p1[1]))
        lat_2 = radians(float(p2[0]))
        lon_2 = radians(float(p2[1]))
        x = (lat_2 - lat_1) * cos((lon_1 + lon_2) / 2)
        y = lon_2 - lon_1
        return sqrt(x * x + y * y) * LengthCalculator.RADIUS_EARTH
