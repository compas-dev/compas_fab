from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from math import pi

from compas import IPY

from compas_robots.model import Joint


from compas_fab.backends import MPMaxJumpError


if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from typing import Dict  # noqa: F401
        from typing import List  # noqa: F401


__all__ = [
    "check_max_jump",
]


def check_max_jump(joint_names, joint_types, start_joint_values, end_joint_values, options):
    # type: (List[str], List[int], List[float], List[float], Dict) -> None
    """Check if the joint positions between two configurations exceed the maximum allowed distance.


    Parameters
    ----------
    joint_names : list of str
        List of joint names.
    joint_types : list of int
        List of joint types.
    start_joint_values : list of float
        List of joint values for the start configuration.
    end_joint_values : list of float
        List of joint values for the end configuration.
    options : dict
        Dictionary containing the following key-value pairs:

        - ``"max_jump_prismatic"``: (:obj:`float`, optional)
        - ``"max_jump_revolute"``: (:obj:`float`, optional)

        Just pass the options dictionary from the planning function.

    Returns
    -------
    None

    Raises
    ------
    MPMaxJumpError
        If the joint positions between two configurations exceed the maximum allowed
        distance specified in the options dictionary.

    """
    assert len(joint_names) == len(joint_types) == len(start_joint_values) == len(end_joint_values)
    max_jump_prismatic = options.get("max_jump_prismatic", 0.1)
    max_jump_revolute = options.get("max_jump_revolute", pi / 2)

    for joint_name, joint_type, v1, v2 in zip(joint_names, joint_types, start_joint_values, end_joint_values):
        # If the joint is REVOLUTE we check the angular difference:
        if joint_type in [Joint.REVOLUTE]:
            difference = abs(v1 - v2)
            if difference > max_jump_revolute:
                raise MPMaxJumpError(
                    message="_check_max_jump(): Joint {} (REVOLUTE) jump {} is too large, exceeds 'max_jump_revolute' of {}".format(
                        joint_name, difference, max_jump_revolute
                    ),
                    joint_name=joint_name,
                    joint_type=joint_type,
                    joint_values_a=start_joint_values,
                    joint_values_b=end_joint_values,
                    value_difference=difference,
                    value_threshold=max_jump_revolute,
                )
        # If the joint is CONTINUOUS the angular difference is calculated in the shortest path:
        elif joint_type in [Joint.CONTINUOUS]:
            diff = abs(v1 - v2) % (2 * pi)  # Normalize the difference to 0 to 2*pi
            if diff > pi:
                diff = 2 * pi - diff
            if diff > max_jump_revolute:
                raise MPMaxJumpError(
                    message="_check_max_jump(): Joint {} (CONTINUOUS) jump {} is too large, exceeds 'max_jump_revolute' of {}".format(
                        joint_name, diff, max_jump_revolute
                    ),
                    joint_name=joint_name,
                    joint_type=joint_type,
                    joint_values_a=start_joint_values,
                    joint_values_b=end_joint_values,
                    value_difference=diff,
                    value_threshold=max_jump_revolute,
                )

        if joint_type in [Joint.PRISMATIC, Joint.PLANAR]:
            diff = abs(v1 - v2)
            if diff > max_jump_prismatic:
                raise MPMaxJumpError(
                    message="_check_max_jump(): Joint {} (PRISMATIC/PLANAR) jump {} is too large, exceeds 'max_jump_prismatic' of {}".format(
                        joint_name, diff, max_jump_prismatic
                    ),
                    joint_name=joint_name,
                    joint_type=joint_type,
                    joint_values_a=start_joint_values,
                    joint_values_b=end_joint_values,
                    value_difference=diff,
                    value_threshold=max_jump_prismatic,
                )
