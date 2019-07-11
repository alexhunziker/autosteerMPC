import math


class Edge(object):
    """Defines an edge for the dijkstra way-planning algorithm"""

    def __init__(self, weight: float, children: list, way_points, start: str = ""):
        self.weight = weight
        self.children = children
        self.start = start
        self.way_points = way_points
        self.reachable_in = math.inf
        self.consumed = False
        self.way_to_here = None

    def potentially_update(self, reached_in: float, way_to_here: []):
        if self.reachable_in > reached_in:
            self.reachable_in = reached_in
            self.way_to_here = way_to_here
            self.consumed = False

    def add_child(self, child):
        self.children.append(child)

    def is_target(self, target_name: str):
        return self.start == target_name

    def consume(self, cut_off):
        self.consumed = True
        weight_after = self.reachable_in + self.weight
        current_way = self.way_to_here + self.way_points
        if weight_after > cut_off:
            return list()

        for child in self.children:
            child.potentially_update(weight_after, current_way)
        return self.children
