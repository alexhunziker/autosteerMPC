import os


class ResultWriter(object):
    def __init__(self, target):
        self.target = target

    def write(self, way_list):
        if os.path.exists(self.target):
            os.remove(self.target)
        file = open(self.target, 'a')
        for point in way_list:
            file.write(point[0] + ',' + point[1] + "\n")
        file.close()
