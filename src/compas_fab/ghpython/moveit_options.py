"""A small wireable options object for the MoveIt Planner Grasshopper component."""

from typing import Optional


class MoveItPlannerOptions:
    """Advanced load options for the `MoveIt Planner` Grasshopper component.

    Produced by the `MoveIt Planner Options` component and consumed by the
    `MoveIt Planner` component. Bundling the rarely-changed parameters into one
    object keeps them off the planner component in the common case, where no
    options need to be set at all.

    This is a plain (non-iterable) class on purpose: a `dict` or `namedtuple`
    would be exploded into its keys/items as it travels over a Grasshopper wire,
    whereas a class instance crosses the wire as a single opaque object.

    Every field is optional; an unset field falls back to the backend default in
    [`RosClient.load_robot_cell`][compas_fab.backends.RosClient.load_robot_cell].

    Parameters
    ----------
    urdf_param_name
        ROS parameter / topic name for the URDF. Defaults to ``/robot_description``.
    srdf_param_name
        ROS parameter / topic name for the SRDF. Defaults to
        ``/robot_description_semantic``.
    http_file_server_base_url
        ROS 2 only: base URL of the HTTP file server hosting ``package://`` meshes.
    """

    def __init__(
        self,
        urdf_param_name: Optional[str] = None,
        srdf_param_name: Optional[str] = None,
        http_file_server_base_url: Optional[str] = None,
    ):
        self.urdf_param_name = urdf_param_name
        self.srdf_param_name = srdf_param_name
        self.http_file_server_base_url = http_file_server_base_url

    def to_load_kwargs(self) -> dict:
        """Return only the set fields as kwargs for `RosClient.load_robot_cell`."""
        kwargs = {}
        if self.urdf_param_name:
            kwargs["urdf_param_name"] = self.urdf_param_name
        if self.srdf_param_name:
            kwargs["srdf_param_name"] = self.srdf_param_name
        if self.http_file_server_base_url:
            kwargs["http_file_server_base_url"] = self.http_file_server_base_url
        return kwargs

    def __repr__(self) -> str:
        return "MoveItPlannerOptions(urdf_param_name={!r}, srdf_param_name={!r}, http_file_server_base_url={!r})".format(
            self.urdf_param_name, self.srdf_param_name, self.http_file_server_base_url
        )
