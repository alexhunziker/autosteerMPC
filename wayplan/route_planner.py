import math

from .data_loader import DataLoader
from .result_writer import ResultWriter


class RoutePlanner(object):
    def __init__(self):
        self.edges = DataLoader("../wayplan/resources/way_graph_raw.txt").load()

    def calculate(self, start: str, destination: str):
        best_destination_weight = math.inf
        potential_solution = None

        open_edges: [] = self.add_starting_edges(start)

        while len(open_edges) > 0:
            best_open_idx = self.find_best_open(open_edges)
            if not best_open_idx:
                break
            best_open = open_edges.pop(best_open_idx)

            if best_open.is_target(destination):
                if best_open.reachable_in < best_destination_weight:
                    best_destination_weight = best_open.reachable_in
                    potential_solution = best_open

            children = best_open.consume(best_destination_weight)
            new_open = set(children) - set(open_edges)
            open_edges = open_edges + list(new_open)

        print(potential_solution.way_to_here)
        print("Reachable in:", potential_solution.reachable_in)
        ResultWriter('../target_trajectory.txt').write(potential_solution.way_to_here)

    def find_best_open(self, open_edges):
        best_open_weight: float = math.inf
        best_open = None
        for idx in range(len(open_edges)):
            if open_edges[idx].weight < best_open_weight and not open_edges[idx].consumed:
                best_open = idx
        return best_open

    def add_starting_edges(self, start: str):
        start_edges = []
        for edge in self.edges:
            if edge.start == start:
                edge.potentially_update(0, [])
                start_edges.append(edge)
        return start_edges


RoutePlanner().calculate("0x1d", "0x1b0")
