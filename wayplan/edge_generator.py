from .edge import Edge
from .length_calculator import LengthCalculator
from .name_generator import NameGenerator


class EdgeGenerator(object):
    def __init__(self):
        self.current = None
        self.name_generator = NameGenerator()
        self.length_calculator = LengthCalculator()

    def set_current(self, lat, lon):
        self.current = (lat, lon)

    def reset_current(self):
        self.current = None

    def add_both_directions(self, lat, lon):
        second_point = (lat, lon)
        return [self.create_edge(self.current, second_point), self.create_edge(second_point, self.current)]

    def create_edge(self, p1, p2):
        name = self.name_generator.generate_for(p1)
        length = self.length_calculator.estimate(p1, p2)
        return Edge(length, start=name, children=[], way_points=[p1, p2])

    def process(self, lat, lon):
        if self.current:
            generated = self.add_both_directions(lat, lon)
            self.current = (lat, lon)
            return generated
        else:
            self.set_current(lat, lon)
            return []
