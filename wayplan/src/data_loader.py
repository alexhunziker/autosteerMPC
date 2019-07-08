from edge import Edge


class DataLoader(object):

    def __init__(self):
        pass

    def load(self):
        a = Edge(5, start="start")
        b = Edge(2, start="start")
        c = Edge(2)
        d = Edge(10)
        e = Edge(1, start="destination")

        a.add_child(c)
        b.add_child(d)
        c.add_child(e)
        d.add_child(e)
        e.add_child(d)

        return [a, b, c, d, e]
