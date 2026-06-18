from typing import TYPE_CHECKING
from typing import Optional

from compas.utilities import await_callback

from compas_fab.backends.exceptions import CollisionCheckError
from compas_fab.backends.interfaces import CheckCollision
from compas_fab.backends.ros.messages import GetStateValidityRequest
from compas_fab.backends.ros.messages import GetStateValidityResponse
from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import JointState
from compas_fab.backends.ros.messages import MultiDOFJointState
from compas_fab.backends.ros.messages import RobotState
from compas_fab.backends.ros.service_description import ServiceDescription

if TYPE_CHECKING:
    from compas_fab.robots import RobotCellState

__all__ = [
    "MoveItCheckCollision",
]


class MoveItCheckCollision(CheckCollision):
    """Callable to check a robot cell state for collisions using MoveIt.

    Backed by MoveIt's ``/check_state_validity`` service
    (``moveit_msgs/GetStateValidity``), which runs a stateless collision +
    constraint check of the given configuration against the planning scene
    currently loaded into ``move_group`` - no planning, no IK.
    """

    GET_STATE_VALIDITY = ServiceDescription(
        "/check_state_validity",
        "GetStateValidity",
        GetStateValidityRequest,
        GetStateValidityResponse,
    )

    def check_collision(self, robot_cell_state: "RobotCellState", options: Optional[dict] = None):
        """Check whether ``robot_cell_state`` is collision-free in MoveIt's planning scene.

        Parameters
        ----------
        robot_cell_state
            The robot cell state to check. Its ``robot_configuration`` is checked,
            and its tool / rigid-body attachment is reflected in the planning scene.
        options
            Dictionary containing the following key-value pairs:

            - ``"group"``: (:obj:`str`, optional) The planning group to check.
              Defaults to the robot's main planning group.

        Returns
        -------
        ``None``
            Returns ``None`` if the state is collision-free.

        Raises
        ------
        :class:`compas_fab.backends.exceptions.CollisionCheckError`
            If the state is in collision (or otherwise invalid). The message lists the
            colliding body pairs reported by MoveIt, and ``collision_pairs`` carries them.
        """
        options = options or {}
        planner = self  # type: MoveItPlanner
        client = planner.client  # type: RosClient
        robot_cell = client.robot_cell  # type: RobotCell
        group = options.get("group") or robot_cell.main_group_name

        # Push the state (configuration + attachments) so the planning scene matches
        # what we are checking. The validity request below then only carries the joint
        # configuration; attachments live in the scene.
        planner.set_robot_cell_state(robot_cell_state)

        configuration = robot_cell.zero_full_configuration().merged(robot_cell_state.robot_configuration)

        kwargs = {
            "configuration": configuration,
            "group": group,
            "errback_name": "errback",
        }
        valid, contacts = await_callback(self._check_collision_async, **kwargs)

        if valid:
            return

        pairs = [(c.get("contact_body_1"), c.get("contact_body_2")) for c in contacts]
        if pairs:
            listed = "\n".join("  '{}' <-> '{}'".format(a, b) for a, b in pairs)
            message = "Robot cell state is in collision. Colliding pairs:\n{}".format(listed)
        else:
            # MoveIt can report invalid without populated contacts (e.g. constraint
            # violations, or contact reporting disabled in the scene).
            message = "Robot cell state is invalid (in collision or violates a constraint)."
        raise CollisionCheckError(message, pairs)

    def _check_collision_async(self, callback, errback, configuration, group):
        """Asynchronous handler of MoveIt's /check_state_validity service."""
        header = Header()
        joint_state = JointState(name=configuration.joint_names, position=configuration.joint_values, header=header)
        # `is_diff=True` keeps the attached collision objects already set on the scene.
        robot_state = RobotState(joint_state, MultiDOFJointState(header=header), is_diff=True)

        def handle(response):
            callback((response.valid, response.contacts))

        # The tuple holds the GetStateValidityRequest *constructor args*; the service
        # description builds the request and runs distro field-filtering on it.
        self.GET_STATE_VALIDITY(self.client, (robot_state, group), handle, errback)
