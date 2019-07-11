import math

from data_loader import DataLoader


class RoutePlanner(object):
    def __init__(self):
        self.edges = DataLoader().load()

    def calculate(self, start: str, destination: str):
        open_edges = []
        best_destination_weight = math.inf
        potential_solution = None

        for edge in self.edges:
            if edge.start == start:
                edge.potentially_update(0, [])
                open_edges.append(edge)

        while len(open_edges) > 0:
            best_open_idx = self.find_best_open(open_edges)
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


    def find_best_open(self, open_edges):
        best_open_weight: float = math.inf
        best_open = None
        for idx in range(len(open_edges)):
            if open_edges[idx].weight < best_open_weight and not open_edges[idx].consumed:
                best_open = idx
        return best_open


RoutePlanner().calculate("start", "destination")
