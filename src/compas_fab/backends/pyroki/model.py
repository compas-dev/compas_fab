import math
from dataclasses import dataclass

from compas_robots.model import Joint

from .conversions import frame_from_wxyz_xyz
from .conversions import frame_to_wxyz_xyz

SUPPORTED_JOINT_TYPES = (Joint.REVOLUTE, Joint.CONTINUOUS, Joint.PRISMATIC, Joint.FIXED)


def _load_pyroki():
    try:
        import jax.numpy as jnp
        import jaxls
        import pyroki
        from pyroki._robot_urdf_parser import JointInfo
        from pyroki._robot_urdf_parser import LinkInfo
    except ImportError as error:
        raise ImportError('The PyRoKI backend requires optional dependencies. Install them with pip install "compas_fab[pyroki]".') from error

    return pyroki, jnp, jaxls, JointInfo, LinkInfo


def _joint_bounds(joint, bounds_by_name):
    if joint.type == Joint.FIXED:
        return 0.0, 0.0
    if joint.limit:
        return float(joint.limit.lower), float(joint.limit.upper)
    if joint.type == Joint.CONTINUOUS:
        return -math.pi, math.pi
    if joint.mimic and joint.mimic.joint in bounds_by_name:
        lower, upper = bounds_by_name[joint.mimic.joint]
        values = (
            joint.mimic.multiplier * lower + joint.mimic.offset,
            joint.mimic.multiplier * upper + joint.mimic.offset,
        )
        return min(values), max(values)
    raise ValueError("Joint '{}' has no limits.".format(joint.name))


def _joint_velocity_limit(joint, velocity_by_name):
    if joint.type == Joint.FIXED:
        return 0.0
    if joint.limit and joint.limit.velocity:
        return float(joint.limit.velocity)
    if joint.mimic and joint.mimic.joint in velocity_by_name:
        return abs(joint.mimic.multiplier) * velocity_by_name[joint.mimic.joint]
    return math.inf


def _topological_joint_order(parent_indices, mimic_source_indices):
    remaining = list(range(len(parent_indices)))
    processed = set()
    order = []

    while remaining:
        ready = [
            index
            for index in remaining
            if (parent_indices[index] == -1 or parent_indices[index] in processed) and (mimic_source_indices[index] == -1 or mimic_source_indices[index] in processed)
        ]
        if not ready:
            raise ValueError("Robot model contains a cyclic kinematic or mimic dependency.")
        for index in ready:
            remaining.remove(index)
            processed.add(index)
            order.append(index)

    return tuple(order)


def _configuration_values(configuration, joint_names):
    if configuration is None:
        return None

    missing = [name for name in joint_names if name not in configuration.keys()]
    if missing:
        raise ValueError("Configuration is missing joints: {}".format(", ".join(missing)))
    return [float(configuration[name]) for name in joint_names]


@dataclass(frozen=True)
class PyRokiModel:
    """Internal bridge between a COMPAS ``RobotModel`` and a PyRoKI robot.

    The wrapped PyRoKI object is deliberately kept out of the public FAB robot
    model. Joint and link names preserve COMPAS ordering so configurations and
    poses can cross the backend boundary without URDF serialization.
    """

    robot: object
    joint_names: tuple
    link_names: tuple

    def configuration_values(self, configuration):
        """Return a configuration in the ordering expected by PyRoKI."""
        return _configuration_values(configuration, self.joint_names)

    def forward_kinematics(self, configuration, link_name=None):
        """Calculate a link frame in the COMPAS robot's base coordinates."""
        _, jnp, _, _, _ = _load_pyroki()
        values = self.configuration_values(configuration)
        poses = self.robot.forward_kinematics(jnp.asarray(values))
        link_name = link_name or self.link_names[-1]
        try:
            link_index = self.link_names.index(link_name)
        except ValueError:
            raise ValueError("Link '{}' does not exist in the PyRoKI model.".format(link_name))
        return frame_from_wxyz_xyz(poses[link_index])


def robot_model_to_pyroki(robot_model, default_configuration=None):
    """Build a PyRoKI kinematic tree directly from a COMPAS robot model.

    No URDF is written or parsed. This function is the sole compatibility
    boundary around PyRoKI's currently private ``JointInfo`` and ``LinkInfo``
    constructors.

    Parameters
    ----------
    robot_model : :class:`compas_robots.RobotModel`
        Robot model to convert.
    default_configuration : :class:`compas_robots.Configuration`, optional
        Default value used by PyRoKI for its joint optimization variable.

    Returns
    -------
    :class:`PyRokiModel`
        Converted model and its COMPAS/PyRoKI ordering metadata.
    """
    pyroki, jnp, jaxls, JointInfo, LinkInfo = _load_pyroki()

    joints = list(robot_model.joints)
    links = list(robot_model.links)
    joint_names = tuple(robot_model.get_configurable_joint_names())
    actuated_index = {name: index for index, name in enumerate(joint_names)}
    joint_index = {joint.name: index for index, joint in enumerate(joints)}
    child_joint = {joint.child.link: joint for joint in joints}

    unsupported = [joint.name for joint in joints if joint.type not in SUPPORTED_JOINT_TYPES]
    if unsupported:
        raise ValueError("PyRoKI does not support joint types used by: {}".format(", ".join(unsupported)))

    twists = []
    parent_transforms = []
    parent_indices = []
    actuated_indices = []
    mimic_act_indices = []
    mimic_source_indices = []
    mimic_multiplier = []
    mimic_offset = []

    for joint in joints:
        axis = [float(joint.axis.x), float(joint.axis.y), float(joint.axis.z)]
        if joint.type in (Joint.REVOLUTE, Joint.CONTINUOUS):
            twist = [0.0, 0.0, 0.0] + axis
        elif joint.type == Joint.PRISMATIC:
            twist = axis + [0.0, 0.0, 0.0]
        else:
            twist = [0.0] * 6
        twists.append(twist)
        parent_transforms.append(frame_to_wxyz_xyz(joint.origin))

        parent_joint = child_joint.get(joint.parent.link)
        parent_indices.append(joint_index[parent_joint.name] if parent_joint else -1)

        if joint.mimic:
            source_name = joint.mimic.joint
            if source_name not in actuated_index:
                raise ValueError("Mimic joint '{}' must reference an independent configurable joint.".format(joint.name))
            actuated_indices.append(-1)
            mimic_act_indices.append(actuated_index[source_name])
            mimic_source_indices.append(joint_index[source_name])
            mimic_multiplier.append(float(joint.mimic.multiplier))
            mimic_offset.append(float(joint.mimic.offset))
        elif joint.name in actuated_index:
            actuated_indices.append(actuated_index[joint.name])
            mimic_act_indices.append(-1)
            mimic_source_indices.append(-1)
            mimic_multiplier.append(1.0)
            mimic_offset.append(0.0)
        else:
            actuated_indices.append(-1)
            mimic_act_indices.append(-1)
            mimic_source_indices.append(-1)
            mimic_multiplier.append(1.0)
            mimic_offset.append(0.0)

    bounds_by_name = {}
    velocity_by_name = {}
    for name in joint_names:
        joint = robot_model.get_joint_by_name(name)
        bounds_by_name[name] = _joint_bounds(joint, bounds_by_name)
        velocity_by_name[name] = _joint_velocity_limit(joint, velocity_by_name)

    lower_limits_all = []
    upper_limits_all = []
    velocity_limits_all = []
    for joint_index_in_topology in _topological_joint_order(parent_indices, mimic_source_indices):
        joint = joints[joint_index_in_topology]
        bounds_by_name[joint.name] = _joint_bounds(joint, bounds_by_name)
        velocity_by_name[joint.name] = _joint_velocity_limit(joint, velocity_by_name)
    for joint in joints:
        lower, upper = bounds_by_name[joint.name]
        lower_limits_all.append(lower)
        upper_limits_all.append(upper)
        velocity_limits_all.append(velocity_by_name[joint.name])

    lower_limits = [bounds_by_name[name][0] for name in joint_names]
    upper_limits = [bounds_by_name[name][1] for name in joint_names]
    velocity_limits = [velocity_by_name[name] for name in joint_names]

    default_values = _configuration_values(default_configuration, joint_names)
    if default_values is None:
        default_values = []
        for lower, upper in zip(lower_limits, upper_limits):
            default_values.append(0.0 if lower <= 0.0 <= upper else (lower + upper) / 2.0)
    default_values = jnp.asarray(default_values, dtype=float)

    class JointVar(jaxls.Var, default_factory=lambda: default_values):
        pass

    joint_info = JointInfo(
        num_joints=len(joints),
        num_actuated_joints=len(joint_names),
        names=tuple(joint.name for joint in joints),
        actuated_names=joint_names,
        twists=jnp.asarray(twists),
        parent_transforms=jnp.asarray(parent_transforms),
        parent_indices=tuple(parent_indices),
        actuated_indices=tuple(actuated_indices),
        lower_limits=jnp.asarray(lower_limits),
        upper_limits=jnp.asarray(upper_limits),
        velocity_limits=jnp.asarray(velocity_limits),
        lower_limits_all=jnp.asarray(lower_limits_all),
        upper_limits_all=jnp.asarray(upper_limits_all),
        velocity_limits_all=jnp.asarray(velocity_limits_all),
        mimic_multiplier=jnp.asarray(mimic_multiplier),
        mimic_offset=jnp.asarray(mimic_offset),
        mimic_act_indices=tuple(mimic_act_indices),
        _topo_sort_inv=_topological_joint_order(parent_indices, mimic_source_indices),
    )
    link_info = LinkInfo(
        num_links=len(links),
        names=tuple(link.name for link in links),
        parent_joint_indices=tuple(joint_index[child_joint[link.name].name] if link.name in child_joint else -1 for link in links),
    )

    return PyRokiModel(
        robot=pyroki.Robot(joints=joint_info, links=link_info, joint_var_cls=JointVar),
        joint_names=joint_names,
        link_names=link_info.names,
    )
