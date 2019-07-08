import math

from data_loader import DataLoader


class RoutePlanner(object):
    def __init__(self):
        self.edges = DataLoader().load()

    def calculate(self, start: str, destination: str):
        open_edges = set()
        way_points = []
        parent = None

        for edge in self.edges:
            if edge.start == start:
                edge.update_reachable_in(0)
                open_edges.add(edge)

        while len(open_edges) > 0:
            best_open = self.find_best_open(open_edges)

            if best_open.is_target(destination):
                return way_points  # TODO: Store and continue, then backtrace

            children = best_open.consume()
            new_open = set(children) - open_edges
            open_edges = open_edges + new_open

    def find_best_open(self, open_edges):
        best_open_weight: float = math.inf
        best_open = None
        for open_edge in open_edges:
            if open_edge.weight < best_open_weight and not open_edge.consumed:
                best_open = open_edge
        return best_open
