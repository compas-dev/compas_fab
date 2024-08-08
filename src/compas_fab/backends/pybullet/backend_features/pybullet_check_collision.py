from itertools import combinations

import compas

from compas.geometry import Frame
from compas_fab.backends.interfaces import CheckCollision
from compas_fab.backends import CollisionCheckError

if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Optional  # noqa: F401
        from typing import Dict  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Tuple  # noqa: F401

        from compas_fab.robots import Robot  # noqa: F401
        from compas_robots import Configuration  # noqa: F401
        from compas.geometry import Frame  # noqa: F401
        from compas_fab.backends.interfaces import ClientInterface  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.backends import PyBulletClient  # noqa: F401
        from compas_fab.backends import PyBulletPlanner  # noqa: F401

    import pybullet
from compas_fab.backends.pybullet.const import STATIC_MASS


class PyBulletCheckCollision(CheckCollision):

    # =======================================
    def check_collision(self, state, options=None):
        # type: (RobotCellState, Optional[Dict]) -> None
        """Checks whether the current robot cell state with the given configuration is in collision.

        The collision check involves the following steps:

        1. Check for collisions between each robot link.
        2. Check for collisions between each robot link and each tool.
        3. Check for collisions between each robot link and each rigid body.
        4. Check for collisions between each attached rigid body and all other rigid bodies.
        5. Check for collisions between each tool and each rigid body.

        In each of the above steps, the collision check is skipped if the collision is allowed
        by the semantics of the robot, tool, or rigid body. For details, see in-line comments in code.


        A collision report is returned with the CollisionCheckError exception message.
        Following the fail-fast principle, the exception is raised when the first collision is detected.
        The rest of the collision pairs are therefore not checked.
        Setting the ``full_report`` option to ``True`` will change this behavior, forcing the collision
        check to continue even after a collision is detected. In this case, a detailed report is returned
        with all failing collision pairs. This can be useful for debugging.

        configuration : :class:`compas_fab.robots.Configuration`
            Configuration to be checked for collisions.  If ``None`` is given, the current
            configuration will be checked.  Defaults to ``None``.
        verbose : :obj:`bool`
            When ``True``, additional information is printed.  Defaults to ``False``.
        full_report : :obj:`bool`
            A collision report is returned with the CollisionCheckError exception message.
            Typically, the exception is raised on the first collision detected and the rest of
            the collision pairs are not checked.
            If ``full_report`` is set to ``True``, the exception will be raised after all
            collision pairs are checked, and the report will contain all failing collision pairs.
            Defaults to ``False``.

        Parameters
        ----------
        state : :class:`compas_fab.robots.RobotCellState`
            The robot cell state describing the robot cell.
            The attribute `robot_configuration`, must contain the full configuration
            of the robot corresponding to the planning group.
        options : dict, optional
            Dictionary containing the following key-value pairs:

            - ``"verbose"``: (:obj:`bool`, optional) When ``True``, additional information is printed.
              Note that this significantly reduces performance.
              Defaults to ``False``.
            - ``"full_report"``: (:obj:`bool`, optional) When ``True``, all collision pairs are checked
              even when one encountered a collision.
              Defaults to ``False``.

        Raises
        ------
        CollisionCheckError
            If the robot is in collision. The exception message contains a collision report.
        """
        # Collect options
        options = options or {}
        verbose = options.get("verbose", False)
        full_report = options.get("full_report", False)

        # Housekeeping for intellisense
        planner = self  # type: PyBulletPlanner
        client = planner.client  # type: PyBulletClient
        robot = client.robot  # type: Robot

        # Set the robot cell state
        skip_set_robot_cell_state = options.get("_skip_set_robot_cell_state", False)
        if not skip_set_robot_cell_state:
            # NOTE: For public API perspective, the robot cell state must be provided,
            # but if this function is called by other features of the planner,
            # the state can be set before calling this function.
            # the hidden option `_skip_set_robot_cell_state` is used to skip this step.
            planner.set_robot_cell_state(state)

        # This stores all the collision messages for a detailed collision report
        collision_messages = []

        # Convenience function for printing in verbose mode
        def verbose_print(msg):
            if verbose:
                print(msg)

        # TODO: Investigate whether the following checks can be done in parallel

        collision_pairs = []  # type: List[Tuple]

        # CC Step 1: Between each Robot Link
        link_names = list(client.robot_link_puids.keys())
        for link_1_name, link_2_name in combinations(link_names, 2):
            cc_pair_info = "CC.1 between robot link '{}' and robot link '{}'".format(link_1_name, link_2_name)
            # Allowed collision originates from the robot semantics (SRDF)
            if {link_1_name, link_2_name} in client.unordered_disabled_collisions:
                verbose_print(cc_pair_info + " - SKIPPED (SEMANTICS)")
                continue
            link_1_id = client.robot_link_puids[link_1_name]
            link_2_id = client.robot_link_puids[link_2_name]
            try:
                client._check_collision(
                    client.robot_puid,
                    "robot_" + link_1_name,
                    client.robot_puid,
                    "robot_" + link_2_name,
                    link_1_id,
                    link_2_id,
                )
                verbose_print(cc_pair_info + " - PASS")
            except CollisionCheckError:
                verbose_print(cc_pair_info + " - COLLISION")
                collision_messages.append(cc_pair_info + " - COLLISION")
                collision_pairs.append(
                    (client.robot.model.get_link_by_name(link_1_name), client.robot.model.get_link_by_name(link_2_name))
                )
                if not full_report:  # Fail on first error if full_report is not requested
                    raise CollisionCheckError("\n".join(collision_messages), collision_pairs)

        # CC Step 2: Between each Robot Link and each of the tools
        for link_name, link_id in client.robot_link_puids.items():
            for tool_name, tool_id in client.tools_puids.items():
                tool_state = client.robot_cell_state.tool_states[tool_name]
                cc_pair_info = "CC.2 between robot link '{}' and tool '{}'".format(link_name, tool_name)
                # Skip over hidden tools
                if tool_state.is_hidden:
                    verbose_print(cc_pair_info + " - SKIPPED (HIDDEN)")
                    continue

                # Allowed collision originates from ToolState
                if link_name in tool_state.touch_links:
                    verbose_print(cc_pair_info + " - SKIPPED (ALLOWED TOUCH LINK)")
                    continue
                try:
                    client._check_collision(
                        client.robot_puid, "robot_" + link_name, tool_id, tool_name, link_index_1=link_id
                    )
                    verbose_print(cc_pair_info + " - PASS")

                except CollisionCheckError:
                    verbose_print(cc_pair_info + " - COLLISION")
                    collision_messages.append(cc_pair_info + " - COLLISION")
                    collision_pairs.append(
                        (client.robot.model.get_link_by_name(link_name), client.robot_cell.tool_models[tool_name])
                    )
                    if not full_report:  # Fail on first error if full_report is not requested
                        raise CollisionCheckError("\n".join(collision_messages), collision_pairs)

        # CC Step 3: Between each link and each rigid body
        for link_name, link_id in client.robot_link_puids.items():
            for body_name, body_ids in client.rigid_bodies_puids.items():
                rigid_body_state = client.robot_cell_state.rigid_body_states[body_name]
                cc_pair_info = "CC.3 between robot link '{}' and rigid body '{}'".format(link_name, body_name)
                # Skip over hidden bodies
                if rigid_body_state.is_hidden:
                    verbose_print(cc_pair_info + " - SKIPPED (HIDDEN)")
                    continue
                # Skip over touch_links that are allowed to touch the rigid body
                if link_name in rigid_body_state.touch_links:
                    verbose_print(cc_pair_info + " - SKIPPED (ALLOWED TOUCH LINK)")
                    continue
                # Perform collision check
                for body_id in body_ids:
                    try:
                        client._check_collision(client.robot_puid, "robot_" + link_name, body_id, body_name, link_id)
                        verbose_print(cc_pair_info + " (body_id '{}') - PASS".format(body_id))
                    except CollisionCheckError:
                        verbose_print(cc_pair_info + " (body_id '{}') - COLLISION".format(body_id))
                        collision_messages.append(cc_pair_info + " (body_id '{}') - COLLISION".format(body_id))
                        collision_pairs.append(
                            (
                                client.robot.model.get_link_by_name(link_name),
                                client.robot_cell.rigid_body_models[body_name],
                            )
                        )
                        if not full_report:  # Fail on first error if full_report is not requested
                            raise CollisionCheckError("\n".join(collision_messages), collision_pairs)

        # CC Step 4: Between each attached rigid body and all other rigid body
        for body_name, body_ids in client.rigid_bodies_puids.items():
            cc_pair_info = "CC.4 between attached rigid body '{}' and other rigid bodies".format(body_name)
            # Skip over hidden bodies
            if client.robot_cell_state.rigid_body_states[body_name].is_hidden:
                verbose_print(cc_pair_info + " - SKIPPED (HIDDEN)")
                continue
            # Skip over non-attached bodies because there is no need to check between two static bodies
            if not (
                client.robot_cell_state.rigid_body_states[body_name].attached_to_tool
                or client.robot_cell_state.rigid_body_states[body_name].attached_to_link
            ):
                verbose_print(cc_pair_info + " - SKIPPED (NOT ATTACHED TO LINK)")
                continue
            for other_body_name, other_body_ids in client.rigid_bodies_puids.items():
                # Skip over hidden bodies
                if client.robot_cell_state.rigid_body_states[other_body_name].is_hidden:
                    continue
                # Skip over same body pairs
                if body_name == other_body_name:
                    continue
                # Skip over allowed touch bodies
                if body_name in client.robot_cell_state.rigid_body_states[other_body_name].touch_bodies:
                    verbose_print(cc_pair_info + " - SKIPPED (ALLOWED TOUCH BODY)")
                    continue
                if other_body_name in client.robot_cell_state.rigid_body_states[body_name].touch_bodies:
                    verbose_print(cc_pair_info + " - SKIPPED (ALLOWED TOUCH BODY)")
                    continue
                # Perform collision check
                for body_id in body_ids:
                    for other_body_id in other_body_ids:
                        cc_pair_info = "CC.4 between attached rigid body '{}' and rigid body '{}'".format(
                            body_name, other_body_name
                        )
                        try:
                            client._check_collision(body_id, body_name, other_body_id, other_body_name)
                            verbose_print(cc_pair_info + " - PASS")
                        except CollisionCheckError:
                            verbose_print(cc_pair_info + " - COLLISION")
                            collision_messages.append(cc_pair_info + " - COLLISION")
                            collision_pairs.append(
                                (
                                    client.robot_cell.rigid_body_models[body_name],
                                    client.robot_cell.rigid_body_models[other_body_name],
                                )
                            )
                            if not full_report:  # Fail on first error if full_report is not requested
                                raise CollisionCheckError("\n".join(collision_messages), collision_pairs)

        ## CC Step 5: Between each tool and each rigid body
        for tool_name, tool_id in client.tools_puids.items():
            tool_state = client.robot_cell_state.tool_states[tool_name]
            cc_pair_info = "CC.5 between tool '{}' and rigid bodies".format(tool_name)
            # Skip over hidden tools
            if tool_state.is_hidden:
                verbose_print(cc_pair_info + " - SKIPPED (TOOL HIDDEN)")
                continue
            for body_name, body_ids in client.rigid_bodies_puids.items():
                rigid_body_state = client.robot_cell_state.rigid_body_states[body_name]
                cc_pair_info = "CC.5 between tool '{}' and rigid body '{}'".format(tool_name, body_name)
                # Skip over hidden bodies
                if rigid_body_state.is_hidden:
                    verbose_print(cc_pair_info + " - SKIPPED (RB HIDDEN)")
                    continue
                # Skip over bodies that are attached to the tool
                if rigid_body_state.attached_to_tool == tool_name:
                    verbose_print(cc_pair_info + " - SKIPPED (RB ATTACHED TO TOOL)")
                    continue
                # Skip over touch_bodies that are allowed to touch the RB
                if tool_name in rigid_body_state.touch_bodies:
                    verbose_print(cc_pair_info + " - SKIPPED (TOOL IS TOUCH BODY IN RB)")
                    continue
                # Perform collision check
                for body_id in body_ids:
                    try:
                        client._check_collision(tool_id, tool_name, body_id, body_name)
                        verbose_print(cc_pair_info + " (body_id '{}') - PASS".format(body_id))
                    except CollisionCheckError:
                        verbose_print(cc_pair_info + " (body_id '{}') - COLLISION".format(body_id))
                        collision_messages.append(cc_pair_info + " (body_id '{}') - COLLISION".format(body_id))
                        collision_pairs.append(
                            (client.robot_cell.tool_models[tool_name], client.robot_cell.rigid_body_models[body_name])
                        )
                        if not full_report:  # Fail on first error if full_report is not requested
                            raise CollisionCheckError("\n".join(collision_messages), collision_pairs)

        if collision_messages:
            raise CollisionCheckError("\n".join(collision_messages), collision_pairs)
