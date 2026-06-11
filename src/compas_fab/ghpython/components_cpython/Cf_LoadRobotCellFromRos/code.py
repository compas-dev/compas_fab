# r: compas_fab>=1.1.0
"""
Load a RobotCell from a connected ROS bridge.

Reads URDF and SRDF from the standard parameter / topic names and (optionally)
fetches the mesh geometry. The result is cached in sticky so subsequent canvas
runs do not re-fetch the (potentially large) mesh payload.

Works against ROS 1 (rosbridge param server + fileserver) and ROS 2 (rosbridge
topics + HTTP file server): the RosClient auto-detects the distro and picks
the matching loader.

If you see HTTP 404 errors while loading geometry, your local ROS may be
ROS 1 but missing the `/rosapi/get_ros_version` service AND the
`/rosdistro` parameter, which causes the client to fall back to ROS 2
defaults. Workarounds: (a) set the `/rosdistro` ROS param on your master,
(b) install the rosapi service, or (c) set `load_geometry` to False and use
a robot cell library instead for visualization.

COMPAS FAB v1.1.0
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import create_id
from compas_ghpython import error
from scriptcontext import sticky as st

from compas_fab.ghpython import ensure_boolean_toggle
from compas_fab.ghpython import register_models_into_cell


class LoadRobotCellFromRos(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        ros_client,
        load_geometry: bool,
        urdf_param_name: str,
        srdf_param_name: str,
        http_file_server_base_url: str,
        reload: bool,
        tools,
        rigid_bodies,
    ):
        ensure_boolean_toggle(ghenv.Component, "load_geometry", default=True)  # noqa: F821

        if ros_client is None or not ros_client.is_connected:
            return (None, None, None)

        load_geometry = True if load_geometry is None else load_geometry

        try:
            detected_distro = ros_client.ros_distro.value
        except Exception:
            detected_distro = None

        # Cache only the base cell; tools/rigid bodies are merged fresh on every run so
        # editing them does not require re-fetching the (potentially large) mesh payload.
        key = create_id(  # noqa: F821
            ghenv.Component,  # noqa: F821
            "ros_robot_cell_{}_{}_{}".format(load_geometry, urdf_param_name or "default", srdf_param_name or "default"),
        )
        base_cell = st.get(key)

        # Fetch on the first run (nothing cached yet) or when the user forces a reload.
        # The mesh payload can be large, so we don't re-fetch on every canvas refresh.
        if reload or base_cell is None:
            kwargs = dict(load_geometry=load_geometry)
            if urdf_param_name:
                kwargs["urdf_param_name"] = urdf_param_name
            if srdf_param_name:
                kwargs["srdf_param_name"] = srdf_param_name
            if http_file_server_base_url:
                kwargs["http_file_server_base_url"] = http_file_server_base_url

            try:
                base_cell = ros_client.load_robot_cell(**kwargs)
            except Exception as e:
                # Don't poison the cache; let the user retry by toggling reload or fixing inputs.
                st.pop(key, None)
                msg = str(e)
                if "404" in msg or "Not Found" in msg:
                    error(  # noqa: F821
                        ghenv.Component,  # noqa: F821
                        f"ROS load failed with HTTP 404. Detected ROS distro: '{detected_distro}'.",
                    )
                else:
                    error(ghenv.Component, "ROS load failed: {}".format(e))  # noqa: F821
                return (None, None, detected_distro)
            st[key] = base_cell

        if base_cell is None:
            return (None, None, detected_distro)

        robot_cell = register_models_into_cell(ghenv.Component, base_cell, tools, rigid_bodies)  # noqa: F821
        robot_cell_state = robot_cell.default_cell_state()
        return (robot_cell, robot_cell_state, detected_distro)
