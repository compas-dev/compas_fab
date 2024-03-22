from compas.geometry import Transformation
from compas.geometry import Frame


def from_tcf_to_t0cf(tcf_frame_in_wcf, tool_coordinate_frame):
    """Converts a frame describing the robot's tool tip (tcf frame) relative to WCF
    to a frame describing the robot's flange (tool0 frame), relative to WCF.

    Let: W_TCF = tcf_frame_in_wcf
    Let: T0CF_TCF = tool_coordinate_frame
    And: TCF_T0CF = (T0CF_TCF)^-1
    Let: frames_t0cf = W_T0CF
    Then: W_T0CF = W_TCF * TCF_T0CF

    Parameters
    ----------
    tcf_frame_in_wcf : :class:`~compas.geometry.Frame`
        Frame (in WCF) of the robot's tool tip (tcf).
    tool_coordinate_frame : :class:`~compas.geometry.Frame`
        Tool tip coordinate frame relative to the flange of the robot.

    Returns
    -------
    :class:`~compas.geometry.Frame`
        Frame (in WCF) of the robot's flange (tool0).

    """
    T0CF_TCF = tool_coordinate_frame
    TCF_T0CF = Transformation.from_frame(T0CF_TCF).inverse()
    W_TCF = Transformation.from_frame(tcf_frame_in_wcf)
    return Frame.from_transformation(W_TCF * TCF_T0CF)
