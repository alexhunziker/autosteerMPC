import math
import random

from .data_loader import DataLoader
from .result_writer import ResultWriter


class RoutePlanner(object):
    def __init__(self):
        #self.edges = DataLoader("../wayplan/resources/way_graph_raw.txt").load()
        self.edges = DataLoader("../wayplan/resources/mini_map_ch.txt").load()

    def calculate(self, start: str, destination: str):
        best_destination_weight = math.inf
        potential_solution = None

        open_edges: [] = self.add_starting_edges(start)

        while len(open_edges) > 0:
            best_open_idx = self.find_best_open(open_edges)
            if best_open_idx is None:
                break
            best_open = open_edges.pop(best_open_idx)

            if best_open.is_target(destination):
                if best_open.reachable_in < best_destination_weight:
                    best_destination_weight = best_open.reachable_in
                    potential_solution = best_open

            children = best_open.consume(best_destination_weight)
            new_open = set(children) - set(open_edges)
            open_edges = open_edges + list(new_open)

        print("DEBUG: Found path:", potential_solution.way_to_here)
        print("INFO: Destination Reachable in:", potential_solution.reachable_in)
        ResultWriter('../target_trajectory.txt').write(potential_solution.way_to_here)
        return potential_solution.way_to_here

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

if __name__ == "__main__":
    for i in range(20):
        print("Calculating route", i)
        p1 = str(hex(int(random.random()*463)))
        p2 = str(hex(int(random.random()*463)))
        print("...from point", p1, "to", p2)
        try:
            result = RoutePlanner().calculate(p1, p2)
            ResultWriter('../target_trajectory'+str(i)+'.txt').write(result)
        except:
            print("No valid route found")
            i = i-1
