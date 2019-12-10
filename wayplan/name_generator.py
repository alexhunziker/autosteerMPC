class NameGenerator(object):
    def __init__(self):
        self.name_dict = dict()
        self.counter = 0

    def generate_for(self, point):
        if point in self.name_dict:
            return self.name_dict[point]
        else:
            name = str(hex(self.counter))
            self.counter += 1
            self.name_dict[point] = name
            print("DEBUG: Coordinate loaded:", name, point)
            return name
