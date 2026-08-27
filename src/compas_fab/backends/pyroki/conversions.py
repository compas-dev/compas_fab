from compas.geometry import Frame
from compas.geometry import Quaternion


def frame_to_wxyz_xyz(frame):
    """Convert a COMPAS frame to PyRoKI's ``wxyz_xyz`` pose layout."""
    quaternion = Quaternion.from_frame(frame)
    return [
        quaternion.w,
        quaternion.x,
        quaternion.y,
        quaternion.z,
        frame.point.x,
        frame.point.y,
        frame.point.z,
    ]


def frame_from_wxyz_xyz(pose):
    """Convert PyRoKI's ``wxyz_xyz`` pose layout to a COMPAS frame."""
    values = [float(value) for value in pose]
    if len(values) != 7:
        raise ValueError("A PyRoKI pose must contain seven values (wxyz_xyz).")
    return Frame.from_quaternion(values[:4], point=values[4:])
