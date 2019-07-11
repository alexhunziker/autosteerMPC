from edge import Edge


class DataLoader(object):

    def __init__(self):
        pass

    def load(self):
        a = Edge(5, start="start", children=[], way_points=[(1, 1), (2, 2), (3, 3)])
        b = Edge(2, start="start", children=[], way_points=[(2, 3), (2, 3)])
        c = Edge(2, start="c", children=[], way_points=[(4, 4)])
        d = Edge(10, start="d", children=[], way_points=[(2, 3), (2, 3)])
        e = Edge(1, start="destination", children=[], way_points=[(9, 9), (9, 9)])

        a.add_child(c)
        b.add_child(d)
        c.add_child(e)
        d.add_child(e)
        e.add_child(a)

        return [a, b, c, d, e]
