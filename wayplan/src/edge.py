import math


class Edge(object):
    """Defines an edge for the dijkstra way-planning algorithm"""

    def __init__(self, weight: float, children: list = [], start: str = "", way_points=[]):
        self.weight = weight
        self.children = children
        self.start = start
        self.way_points = way_points
        self.reachable_in = math.inf
        self.consumed = False

    def update_reachable_in(self, reached_in, parent):
        if self.reachable_in > reached_in:
            self.reachable_in = reached_in
            self.parent = parent

    def add_child(self, child):
        self.children.append(child)

    def is_target(self, target_name: str):
        return self.start == target_name

    def consume(self):
        self.consumed = True
        weight_after = self.reachable_in + self.weight
        for child in self.children:
            child.update_reachable_in(weight_after, self)
        return self.children
