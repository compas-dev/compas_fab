
from pybullet_planning import is_connected, disconnect, connect, HideOutput
from pybullet_planning import load_pybullet
from pybullet_planning import get_link_pose, link_from_name, set_joint_positions, joints_from_names
from pybullet_planning import inverse_kinematics

from compas.geometry import Frame
from compas_fab.backends.pybullet.map_pose import pb_pose_from_Frame

DEFAULT_SCALE = 1.

class PybulletClient(object):
    """Interface to use pybullet as backend via the **pybullet_plannning**.

    The connection is managed by ``roslibpy``.

    :class:`.PybulletClient` is a context manager type, so it's best
    used in combination with the ``with`` statement to ensure
    resource deallocation.

    # TODO: in pybullet we only take meter inputs. conversion needs to be done in compas side.

    Parameters
    ----------
    viewer : :obj:`bool`
        Enable pybullet GUI. Defaults to True.

    Examples
    --------

    >>> from compas_fab.backends import PybulletClient
    >>> with PybulletClient() as client:
    ...     print('Connected: %s' % client.is_connected)
    Connected: 1

    Note
    ----
    For more examples, check out the :ref:`ROS examples page <ros_examples>`.
    """

    def __init__(self, viewer=True, verbose=False):
        super(PybulletClient, self).__init__()
        self.viewer = viewer
        self._ik_fn = None
        self._fk_fn = None
        self.verbose = verbose

    def __enter__(self):
        connect(use_gui=self.viewer)
        return self

    def __exit__(self, *args):
        disconnect()

    @property
    def is_connected(self):
        """Indicates whether the client has an active connection.

        Returns:
            bool: True if connected, False otherwise.
        """
        return is_connected()


    # ==========================================================================
    # robot loading
    # ==========================================================================

    def create_pb_robot(self, urdf_filename, fixed_base=True):
        """Create a pybullet robot using the input urdf file.

        Parameters
        ----------
        urdf_filename : [type]
            absolute file path to the urdf file. The mesh file can be linked by either
            `package::` or relative path.
        fixed_base : bool, optional
            True if the robot is fixed-base, by default True

        Returns
        -------
        int
            a pybullet body unique index.
        """
        # TODO: fuse end effector and robot link tree into one
        with HideOutput(not self.verbose):
            pb_robot = load_pybullet(urdf_filename, fixed_base=fixed_base)
        return pb_robot

    # ==========================================================================
    # ik, fk, planning interface
    # ==========================================================================

    def forward_kinematics(self, joint_positions, base_link, group, joint_names, ee_link, pb_robot=None):
        # TODO: base_link transformation
        assert pb_robot is not None, 'pybullet robot must be given!'
        pb_joints = joints_from_names(pb_robot, joint_names)
        pb_ee_link = link_from_name(pb_robot, ee_link)

        set_joint_positions(pb_robot, pb_joints, joint_positions)
        point, quat = get_link_pose(pb_robot, pb_ee_link)

        return Frame.from_quaternion(quat, point=point)

    def inverse_kinematics(self, frame, base_link, group,
                           joint_names, joint_positions, avoid_collisions=True,
                           constraints=None, attempts=200,
                           attached_collision_meshes=None, pb_robot=None, ee_link=None):
        # TODO: base_link transformation
        assert pb_robot is not None, 'pybullet robot must be given!'
        # TODO: check joint limit, pass in custom_limits
        # https://github.com/yijiangh/pybullet_planning/blob/dev/src/pybullet_planning/interfaces/kinematics/ik_utils.py#L39

        pb_ee_link = link_from_name(pb_robot, ee_link)
        kinematic_conf = inverse_kinematics(pb_robot, pb_ee_link, pb_pose_from_Frame(joint_positions), attempts=200)
        return kinematic_conf
