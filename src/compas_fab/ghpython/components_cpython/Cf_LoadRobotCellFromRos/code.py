# r: compas_fab>=1.1.0
"""
Load a RobotCell from a connected ROS bridge.

Reads URDF and SRDF from the standard parameter / topic names and (optionally)
fetches the mesh geometry. The result is cached in sticky so subsequent canvas
runs do not re-fetch the (potentially large) mesh payload.

Works against ROS 1 (rosbridge param server + fileserver) and ROS 2 (rosbridge
topics + HTTP file server) — the RosClient auto-detects the distro and picks
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
from compas_ghpython import create_id
from compas_ghpython import error
from scriptcontext import sticky as st


class LoadRobotCellFromRos(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        ros_client,
        load_geometry: bool,
        urdf_param_name: str,
        srdf_param_name: str,
        http_file_server_base_url: str,
        load: bool,
    ):
        if ros_client is None or not ros_client.is_connected:
            return (None, None, None)

        load_geometry = True if load_geometry is None else load_geometry

        try:
            detected_distro = ros_client.ros_distro.value
        except Exception:
            detected_distro = None

        key = create_id(  # noqa: F821
            ghenv.Component,  # noqa: F821
            "ros_robot_cell_{}_{}_{}".format(load_geometry, urdf_param_name or "default", srdf_param_name or "default"),
        )

        if load:
            kwargs = dict(load_geometry=load_geometry)
            if urdf_param_name:
                kwargs["urdf_param_name"] = urdf_param_name
            if srdf_param_name:
                kwargs["srdf_param_name"] = srdf_param_name
            if http_file_server_base_url:
                kwargs["http_file_server_base_url"] = http_file_server_base_url

            try:
                robot_cell = ros_client.load_robot_cell(**kwargs)
                robot_cell_state = robot_cell.default_cell_state()
                st[key] = (robot_cell, robot_cell_state)
            except Exception as e:
                msg = str(e)
                if "404" in msg or "Not Found" in msg:
                    error(  # noqa: F821
                        ghenv.Component,  # noqa: F821
                        f"ROS load failed with HTTP 404. Detected ROS distro: '{detected_distro}'.",
                    )
                else:
                    error(ghenv.Component, "ROS load failed: {}".format(e))  # noqa: F821
                return (None, None, detected_distro)

        cached = st.get(key)
        if cached is None:
            return (None, None, detected_distro)

        robot_cell, robot_cell_state = cached
        return (robot_cell, robot_cell_state, detected_distro)
