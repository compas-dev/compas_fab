from compas_fab.backends.interfaces import ClientInterface


class AnalyticalKinematicsClient(ClientInterface):
    """Dummy client when running Analytical Inverse Kinematics without a client.

    This client does not have any functionality, it is only used to hold
    """

    def __init__(self, verbose=False):
        # type (str, bool) -> None
        super(AnalyticalKinematicsClient, self).__init__()
        self.verbose = verbose
