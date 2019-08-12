
class Grasp(object):
    def __init__(self, index, num, approach, attach, retreat):
        self.index = index # brick index
        self.num = num # grasp id
        self.approach = approach # compas Frame
        self.attach = attach # compas Frame
        self.retreat = retreat # compas Frame
    def __repr__(self):
        return '{}(b {}, g {})'.format(self.__class__.__name__, self.index, self.num)
