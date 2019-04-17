class MoveIt(object):
    def __init__(self):
        super(MoveIt, self).__init__()

    def run(self):
        print('Running')


class BaseRos(object):
    def __init__(self, port):
        super(BaseRos, self).__init__()
        self.port = port

    def base(self):
        print('Oh base!')


class Ros(BaseRos):
    def __init__(self, port, m=MoveIt):
        super(Ros, self).__init__(port)
        print('Ros' + m.__name__)
        self.__class__ = type('Ros_' + m.__name__, (Ros, m), {})

    # def run(self):
    #     print('Run like shit')


a = Ros(10)
a.base()
a.run()
