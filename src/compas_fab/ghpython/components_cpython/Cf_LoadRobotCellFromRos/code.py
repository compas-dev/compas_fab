# r: compas_fab>=1.1.0
"""
Load a RobotCell from a connected ROS bridge.

Reads URDF and SRDF from the standard parameter / topic names and (optionally)
fetches the mesh geometry. The result is cached in sticky so subsequent canvas
runs do not re-fetch the (potentially large) mesh payload.

Works against ROS 1 (rosbridge param server + fileserver) and ROS 2 (rosbridge
topics + HTTP file server) — the RosClient auto-detects the distro and picks
the matching loader.

COMPAS FAB v1.1.0
"""

import Grasshopper
from compas_ghpython import create_id
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
            return (None, None)

        load_geometry = True if load_geometry is None else load_geometry

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

            robot_cell = ros_client.load_robot_cell(**kwargs)
            robot_cell_state = robot_cell.default_cell_state()
            st[key] = (robot_cell, robot_cell_state)

        cached = st.get(key)
        if cached is None:
            return (None, None)

        return cached
