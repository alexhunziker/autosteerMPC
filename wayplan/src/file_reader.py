from edge_generator import EdgeGenerator


class FileReader(object):
    def __init__(self, file):
        self.file = file
        self.graph = list()
        self.edge_generator = EdgeGenerator()

    def go(self):
        open_file = open(self.file, 'r')
        line = open_file.readline()
        while line:
            line_segments = line.split("\t")
            if line_segments[0] == "T":
                self.process_position(line_segments[1], line_segments[2])
            if line_segments[0] == "type":
                self.reset_current()
            line = open_file.readline()
        print(self.graph)
        return self.graph

    def process_position(self, lat, lon):
        new_edges = self.edge_generator.process(lat, lon)
        # print(new_edges)
        self.graph.extend(new_edges)

    def reset_current(self):
        self.edge_generator.reset_current()

# FileReader("resources/way_graph_raw.txt").go()
