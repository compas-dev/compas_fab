import socket

from compas.geometry import Frame
from compas.geometry import matrix_from_frame

from compas_fab.backends.exceptions import BackendError
from compas_fab.backends.vrep.remote_api import vrep

from compas_fab.robots import Configuration

__all__ = [
    'VrepError',
]

DEFAULT_OP_MODE = vrep.simx_opmode_blocking

# --------------------------------------------------------------------------
# MAPPINGS
# The following mapping functions are only internal to make sure
# all transformations from and to V-REP are consistent
# --------------------------------------------------------------------------


def vrep_pose_to_frame(pose, scale):
    return Frame.from_list(floats_from_vrep(pose, scale))


def frame_to_vrep_pose(frame, scale):
    # COMPAS FAB uses meters, just like V-REP,
    # so in general, scale should always be 1
    pose = matrix_from_frame(frame)
    pose[0][3] = pose[0][3] / scale
    pose[1][3] = pose[1][3] / scale
    pose[2][3] = pose[2][3] / scale
    return pose[0] + pose[1] + pose[2] + pose[3]


def config_from_vrep(list_of_floats, scale):
    # COMPAS FAB uses radians and meters, just like V-REP,
    # so in general, scale should always be 1
    radians = list_of_floats[3:]
    prismatic_values = map(lambda v: v * scale, list_of_floats[0:3])
    return Configuration.from_prismatic_and_revolute_values(prismatic_values, radians)


def config_to_vrep(config, scale):
    # COMPAS FAB uses radians and meters, just like V-REP,
    # so in general, scale should always be 1
    values = list(map(lambda v: v / scale, config.prismatic_values))
    values.extend(config.revolute_values)
    return values


def floats_to_vrep(list_of_floats, scale):
    return [v / scale for v in list_of_floats]


def floats_from_vrep(list_of_floats, scale):
    return [v * scale for v in list_of_floats]


def assert_robot(robot):
    if not robot:
        raise ValueError('No instance of robot found')
    if not robot.model:
        raise ValueError('The robot instance has no model information attached')
    if 'index' not in robot.model.attr:
        raise ValueError('Robot model needs to define an index as part of the model.attr dictionary')

# --------------------------------------------------------------------------
# NETWORKING HELPERS
# A couple of simple networking helpers for host name resolution
# --------------------------------------------------------------------------


def is_ipv4_address(addr):
    try:
        socket.inet_aton(addr)
        return True
    except socket.error:
        return False


def resolve_host(host):
    if is_ipv4_address(host):
        return host
    else:
        return socket.gethostbyname(host)


class VrepError(BackendError):
    """Wraps an exception that occurred inside the simulation engine."""

    def __init__(self, message, error_code):
        super(VrepError, self).__init__('Error code: ' +
                                        str(error_code) +
                                        '; ' + message)
        self.error_code = error_code
