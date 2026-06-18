# r: compas_fab>=1.1.0
"""
Create a MoveItPlanner from a connected RosClient, loading its robot cell from ROS.

Loading and planner setup are one step: MoveIt's planning scene is defined by the
URDF/SRDF loaded into move_group, so the cell can only come from ROS - there is
nothing to load separately and wire in. Tools and rigid bodies wired into
`tools` / `rigid_bodies` are registered into the cell and uploaded as collision
objects.

The base cell is fetched from ROS once and cached in sticky (the mesh payload can
be large); set `reload` to re-fetch. The planner is cached too and rebuilt if the
client changes (e.g. a reconnect). Advanced load parameters live on the optional
`options` input (see the MoveIt Planner Options component); the default case needs
none of them.

If you see HTTP 404 errors while loading geometry, your local ROS may be ROS 1 but
missing the `/rosapi/get_ros_version` service AND the `/rosdistro` parameter, which
makes the client fall back to ROS 2 defaults. Workarounds: set the `/rosdistro` ROS
param, install the rosapi service, or set `load_geometry` to False.

COMPAS FAB v1.1.0
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import create_id
from compas_ghpython import error
from scriptcontext import sticky as st

from compas_fab.backends import MoveItPlanner
from compas_fab.ghpython import ensure_boolean_toggle
from compas_fab.ghpython import register_models_into_cell


class MoveItPlannerComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, ros_client, tools, rigid_bodies, load_geometry: bool, options, reload: bool):
        ensure_boolean_toggle(ghenv.Component, "load_geometry", default=True)  # noqa: F821

        if ros_client is None or not ros_client.is_connected:
            return (None, None, None)

        load_geometry = True if load_geometry is None else load_geometry
        load_kwargs = options.to_load_kwargs() if options is not None else {}

        try:
            detected_distro = ros_client.ros_distro.value
        except Exception:
            detected_distro = None

        # Cache the planner; rebuild if the client changed (e.g. user reconnected).
        planner_key = create_id(ghenv.Component, "moveit_planner")  # noqa: F821
        planner = st.get(planner_key)
        planner_rebuilt = False
        if planner is not None and planner.client is not ros_client:
            planner = None
        if planner is None:
            planner = MoveItPlanner(ros_client)
            st[planner_key] = planner
            planner_rebuilt = True

        # Cache only the base cell; tools/rigid bodies are merged fresh on every run
        # so editing them does not require re-fetching the (large) mesh payload.
        sig = "{}_{}_{}".format(
            load_geometry,
            load_kwargs.get("urdf_param_name"),
            load_kwargs.get("srdf_param_name"),
        )
        cell_key = create_id(ghenv.Component, "ros_robot_cell_" + sig)  # noqa: F821
        base_cell = st.get(cell_key)

        if reload or base_cell is None:
            kwargs = dict(load_geometry=load_geometry)
            kwargs.update(load_kwargs)
            try:
                base_cell = ros_client.load_robot_cell(**kwargs)
            except Exception as e:
                st.pop(cell_key, None)  # don't poison the cache
                msg = str(e)
                if "404" in msg or "Not Found" in msg:
                    error(  # noqa: F821
                        ghenv.Component,  # noqa: F821
                        f"ROS load failed with HTTP 404. Detected ROS distro: '{detected_distro}'.",
                    )
                else:
                    error(ghenv.Component, "ROS load failed: {}".format(e))  # noqa: F821
                return (None, None, detected_distro)
            st[cell_key] = base_cell

        if base_cell is None:
            return (None, detected_distro)

        robot_cell = register_models_into_cell(ghenv.Component, base_cell, tools, rigid_bodies)  # noqa: F821

        # Upload to the planning scene only when the set of registered models
        # changed (a rebuilt planner, or tools/rigid bodies added/removed/renamed),
        # so a plain recompute does not re-upload the scene to MoveIt on every tick.
        # Content-only edits to a same-named model are caught by `reload`.
        cell_sig = robot_cell.structural_signature()
        sig_key = create_id(ghenv.Component, "moveit_cell_sig")  # noqa: F821
        if planner_rebuilt or reload or st.get(sig_key) != cell_sig:
            planner.set_robot_cell(robot_cell)
            st[sig_key] = cell_sig

        return (planner, robot_cell, detected_distro)
