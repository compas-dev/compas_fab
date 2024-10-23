from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas import IPY
from compas import json_load
from compas.datastructures import Mesh
from compas.geometry import Box
from compas.geometry import Cone
from compas.geometry import Frame
from compas_robots import ToolModel

import compas_fab
from compas_fab.robots import RigidBody
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from typing import Optional  # noqa: F401
        from typing import Tuple  # noqa: F401

        from compas_fab.backends.interfaces import ClientInterface  # noqa: F401


__all__ = [
    "RigidBodyLibrary",
    "RobotCellLibrary",
    "ToolLibrary",
]


class ToolLibrary(object):
    """A collection of built-in tools that can be used for testing and demonstration.
    The :class:`compas_robot.ToolModel` objects created by the factory methods
    can be used to write examples, so that the example code can stay short.

    Some of the tools are created programmatically, these usually contain simple shapes such as cones, boxes, etc.
    They are typically faster to load.
    Some of the tools are loaded from URDF, SRDF and local mesh files, similar to the robots.
    This is possible because ToolModel is a subclass of RobotModel and can be loaded in the same way.
    Some of these tools also have a different visual and collision mesh.

    Some of the tools have a kinematic chain, which is used to represent shape-changing tools such as a gripper with jaws.
    These are referred to kinematic tools in the compas_fab library.
    The kinematic chain is represented similar to a RobotModel. The configuration of the kinematic chain can be described
    in the context of a RobotCell using :class:`compas_fab.robots.ToolState` object in :attr:`compas_fab.robots.RobotCellState.tool_states`.

    All the tools are modelled following ROS REP 199 recommendations (`REP 199 https://gavanderhoorn.github.io/rep/rep-0199.html>`_)
    where the tool's base frame is attached to the link named 'flange' in the RobotModel.
    The 'flange' link frame must have its Positive X (x+) point away from the last link.

    Note that this convention is present in many of Robots loaded from URDF files, but not all.
    Be aware that the 'flange' link is not necessary equal to the 'tool0' frame displayed on the robot controller,
    the orientation of the 'tool0' frame is robot-brand-dependent.
    The rationale of using a consistent 'flange' frame is that any tool can be attached to any robot interchangeably
    without additional rotations to align the tool model and the robot flange.

    Examples
    --------

    >>> from compas_fab.robots import ToolLibrary
    >>> tool = ToolLibrary.cone()
    >>> tool.name
    'cone'
    """

    @classmethod
    def cone(cls, load_geometry=True, radius=0.02, length=0.1):
        # type: (Optional[bool], Optional[float], Optional[float]) -> ToolModel
        """Create and return a cone as ToolModel, useful for simulating a drawing tool.

        The cone points towards the positive X-axis of the tool frame.
        The tool has only one visual mesh, which is also used for collision mesh.

        The tool TCF is located at the tip of the cone,
        it is a translation offset from T0CF by its length (default 0.1) along the X-axis of the T0CF.
        The tool name is 'cone'.

        Parameters
        ----------
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the tool.
        radius: :obj:`float`, optional
            Default is `0.02`, which means that the radius of the cone is 2cm.
        length: :obj:`float`, optional
            Default is `0.1`, which means that the length of the cone is 10cm.

        Returns
        -------
        :class:`compas_fab.robots.ToolModel`
            Newly created instance of the tool.
        """
        tool_frame = Frame([length, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])

        if load_geometry:
            cone = Cone(radius, length, Frame.worldYZ())
            tool_mesh = Mesh.from_shape(cone)
            # Do not use the cone.stl because it points towards the Z axis
            # tool_mesh = Mesh.from_stl(compas_fab.get("planning_scene/cone.stl"))
        else:
            tool_mesh = None

        return ToolModel(visual=tool_mesh, frame_in_tool0_frame=tool_frame, name="cone")

    @classmethod
    def printing_tool(cls, load_geometry=True, tool_size=1.0):
        """Create and return a printing tool as ToolModel, useful for simulating a 3D printing tool.

        The Tool Frame is located at the tip of the tool,
        equal to `Frame([0.2, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])`.
        It has the same orientation as its base frame.
        Its Z-axis points out of the printing nozzle into the material being printed.
        Therefor, the printing targets should be defined with the Z-axis pointing towards the object being printed.

        The tool name is 'printing_tool'.

        Changing the `tool_size` parameter will scale the tool.
        The default size is 1.0, corresponding to the tool tip being 1m away (Z-direction) from the base frame.

        Parameters
        ----------
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the tool.
        tool_size: :obj:`float`, optional
            Default is `1.0`, which means that the tool tip is 1m away from the base frame.
        """

        tool_frame = Frame([0.2, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
        tool_frame.scale(tool_size)

        tool_model = ToolModel(visual=None, frame_in_tool0_frame=tool_frame, name="printing_tool")
        meshes = []
        if load_geometry:
            obj = compas_fab.get("tool_library/printing_tool.obj")
            tool_mesh = Mesh.from_obj(obj)
            tool_mesh.scale(tool_size)
            meshes.append(tool_mesh)

        tool_model.add_link("printing_tool_link", visual_mesh=meshes, collision_mesh=meshes)

        return tool_model

    @classmethod
    def static_gripper(cls, load_geometry=True):
        # type: (Optional[bool]) -> ToolModel
        """Create and return a static gripper ToolModel, useful for simulating a gripper.

        The gripper is 0.1m (10cm) thick from base to its gripping face.
        The gripper has two jaws that have an opening width of 0.2m (20cm).
        The tool has three visual meshes representing the gripper body and the jaws.
        The visual mesh is also used for collision mesh.
        The tool TCF is located at the gripper face of the gripper,
        it is a translation offset from T0CF by +0.1 along the X-axis of the T0CF.
        The tool name is 'gripper'.
        Bar material can be held with the gripper with the long axis matching the Z axis of the TCF.

        Parameters
        ----------
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the tool.

        Returns
        -------
        :class:`compas_fab.robots.ToolModel`
            Newly created instance of the tool.
        """
        tool_frame = Frame([0.1, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
        tool_model = ToolModel(visual=None, frame_in_tool0_frame=tool_frame, name="gripper")

        # The following programmatic construction of the gripper assumes the the gripper is
        # opening towards Z axis, this is transformed at the end to match the X axis.
        # Before the transformation,
        # X is the direction of the jaw movement, Y direction is the long axis when holding bar materials.

        meshes = []
        if load_geometry:
            # Transformation to rotate the gripper to match the X axis
            t = Frame([0, 0, 0], [0, 0, 1], [1, 0, 0]).to_transformation().inverted()
            # Create the gripper body
            shape = Box.from_corner_corner_height([-0.2, -0.05, 0], [0.2, 0.05, 0], 0.1)
            meshes.append(Mesh.from_shape(shape).transformed(t))
            # Create the gripper finger
            shape = Box.from_corner_corner_height([-0.2, -0.05, 0.1], [-0.1, 0.05, 0.1], 0.2)
            meshes.append(Mesh.from_shape(shape).transformed(t))
            # Create the gripper finger
            shape = Box.from_corner_corner_height([0.2, -0.05, 0.1], [0.1, 0.05, 0.1], 0.2)
            meshes.append(Mesh.from_shape(shape).transformed(t))
        tool_model.add_link("gripper_body", visual_meshes=meshes, collision_meshes=meshes)

        return tool_model

    @classmethod
    def static_gripper_small(cls, load_geometry=True):
        # type: (Optional[bool]) -> ToolModel
        """Create and return a static gripper ToolModel, useful for simulating a gripper.

        The gripper is 0.05m (5cm) thick from base to its gripping face.
        The gripper has two jaws that have an opening width of 0.05m (5cm).
        The tool has three visual meshes representing the gripper body and the jaws.
        The visual mesh is also used for collision mesh.
        The tool TCF is located at the gripper face of the gripper,
        it is a translation offset from T0CF by +0.1 along the X-axis of the T0CF.
        The tool name is 'gripper'.
        Bar material can be held with the gripper with the long axis matching the Z axis of the TCF.

        Parameters
        ----------
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the tool.

        Returns
        -------
        :class:`compas_fab.robots.ToolModel`
            Newly created instance of the tool.
        """
        tool_frame = Frame([0.05, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
        tool_model = ToolModel(visual=None, frame_in_tool0_frame=tool_frame, name="gripper")

        # The following programmatic construction of the gripper assumes the the gripper is
        # opening towards Z axis, this is transformed at the end to match the X axis.
        # Before the transformation,
        # X is the direction of the jaw movement, Y direction is the long axis when holding bar materials.

        meshes = []
        if load_geometry:
            # Transformation to rotate the gripper to match the X axis
            t = Frame([0, 0, 0], [0, 0, 1], [1, 0, 0]).to_transformation().inverted()
            # Create the gripper body
            shape = Box.from_corner_corner_height([-0.1, -0.05, 0], [0.1, 0.05, 0], 0.05)
            meshes.append(Mesh.from_shape(shape).transformed(t))
            # Create the gripper finger
            shape = Box.from_corner_corner_height([-0.1, -0.05, 0.05], [-0.05, 0.05, 0.05], 0.1)
            meshes.append(Mesh.from_shape(shape).transformed(t))
            # Create the gripper finger
            shape = Box.from_corner_corner_height([0.1, -0.05, 0.05], [0.05, 0.05, 0.05], 0.1)
            meshes.append(Mesh.from_shape(shape).transformed(t))
        tool_model.add_link("gripper_body", visual_meshes=meshes, collision_meshes=meshes)

        return tool_model

    @classmethod
    def kinematic_gripper(cls, load_geometry=True):
        # type: (Optional[bool]) -> ToolModel
        """Create and return a kinematic gripper ToolModel, useful for simulating a gripper with jaws.

        The gripper is 0.05m (5cm) thick from base to its gripping face.
        The tool has a total of three links representing the gripper body 'body'
        and the two jaws `left_finger` and `right_finger`.

        The gripper has two movable fingers that are 0.1m (10cm) long.
        The fingers can be moved by changing the tool's configuration.
        They are represented by two joints `'left_finger_joint'` and `'right_finger_joint'` each with limits [0.0, 0.025].
        The closed state has a position [0,0], and the fingers are 0.05m (5cm) apart.
        The open state has a position [0.025, 0.025], and the fingers are 0.1m (10cm) apart.

        The tool has the same visual and collision mesh.
        The tool's TCF is located at the tip of the gripper fingers,
        which is a translation offset from T0CF by +0.15 along the X-axis of the T0CF.
        The tool name is 'gripper'.
        Bar material can be held with the gripper if the long axis of the bar matches the Z axis of the TCF.

        Parameters
        ----------
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the tool.

        Returns
        -------
        :class:`compas_fab.robots.ToolModel`
            Newly created instance of the tool.
        """

        tool_model = json_load(compas_fab.get("tool_library/kinematic_gripper/gripper.json"))
        assert isinstance(tool_model, ToolModel)
        return tool_model


class RigidBodyLibrary(object):

    @classmethod
    def target_marker(cls, size=1.0):
        # type: (Optional[float]) -> RigidBody
        """Create and return a target marker as RigidBody, useful for visualizing the target pose.

        The target marker points out the X, Y, Z directions with its shape.
        It is fully contained within a cube in the positive octant of the target frame.
        The size of the cube is determined by the input size.

        The marker has only one visual mesh, and no collision mesh.
        """

        # Load the target marker mesh
        obj = compas_fab.get("planning_scene/target_marker.obj")
        mesh = Mesh.from_obj(obj)

        # Scale the target marker to the desired size
        mesh.scale(size)

        return RigidBody(mesh, None)


class RobotCellLibrary(object):
    """A collection of built-in robot cells that can be used for testing and demonstrations.
    The :class:`compas_fab.robots.RobotCell` and :class:`compas_fab.robots.RobotCellState`
    objects created by the factory methods can be used to write examples,
    so that the example code can stay short.

    The robot cells with only a robot name (e.g. 'ur5') contains only the robot model and semantics.
    Some robot cells contain also Tool(s) and Rigid Body(s).
    The Tool(s) are often loaded from the ToolLibrary.
    Other collision objects such as robot backpacks, floors, tables, workpieces, etc. may also be included
    depending on the specific robot cell.

    All the robot cells constructors has a `load_geometry` parameter that can be used to
    decide if the geometry for the robot and the tool(s) should be loaded or not.
    In order to visualize the robot cell or to perform planning functions, the geometry must be loaded.
    However, for some tests that may not require geometry, setting `load_geometry` to `'false'`
    can speed up the test. If in doubt, use the default value `True`.

    The RobotCellState object contains a predefined state that match with the robot cell.
    The tool(s) and workpiece(s) may or may not be attached depending on the specific robot cell scenario.
    Users can simply modify the state to create different scenarios.

    Examples
    --------

    >>> from compas_fab.robots import RobotCellLibrary
    >>> robot_cell, robot_cell_state = RobotCellLibrary.ur5_cone_tool()
    >>> robot_cell.robot.name
    'ur5_robot'
    >>> robot_cell_state.get
    """

    # ---------------------------------------------------------------------
    # Robot Cells with only Robots
    # ---------------------------------------------------------------------

    @classmethod
    def rfl(cls, load_geometry=True):
        # type: (Optional[bool]) -> Tuple[RobotCell, RobotCellState]
        """Create and return the RFL robot with 4 ABB irb 4600 and twin-gantry setup.

        The returned :class:`compas_fab.robots.Robot` object contains the robot model and semantics.

        Parameters
        ----------
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the robot.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            Newly created instance of the robot.
        """

        robot_cell = RobotCell.from_urdf_and_srdf(
            urdf_filename=compas_fab.get("robot_library/rfl/urdf/robot_description.urdf"),
            srdf_filename=compas_fab.get("robot_library/rfl/robot_description_semantic.srdf"),
            local_package_mesh_folder="robot_library/rfl" if load_geometry else None,
        )
        robot_cell_state = RobotCellState.from_robot_cell(robot_cell)

        return robot_cell, robot_cell_state

    @classmethod
    def ur5(cls, load_geometry=True):
        # type: (Optional[bool]) -> Tuple[RobotCell, RobotCellState]
        """Returns a UR5 robot.

        The returned :class:`compas_fab.robots.Robot` object contains the robot model and semantics.

        The main planning group of the robot is named 'manipulator'.
        The first and last link on the 'manipulator' group is named 'base_link' and 'tool0'.

        Parameters
        ----------
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the robot.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            Newly created instance of the robot.
        """

        robot_cell = RobotCell.from_urdf_and_srdf(
            urdf_filename=compas_fab.get("robot_library/ur5_robot/urdf/robot_description.urdf"),
            srdf_filename=compas_fab.get("robot_library/ur5_robot/robot_description_semantic.srdf"),
            local_package_mesh_folder="robot_library/ur5_robot" if load_geometry else None,
        )
        robot_cell_state = RobotCellState.from_robot_cell(robot_cell)

        return robot_cell, robot_cell_state

    @classmethod
    def ur10e(cls, load_geometry=True):
        # type: (Optional[bool]) -> Tuple[RobotCell, RobotCellState]
        """Returns a UR10e robot.

        The returned :class:`compas_fab.robots.Robot` object contains the robot model and semantics.

        Parameters
        ----------
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the robot.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            Newly created instance of the robot.
        """

        robot_cell = RobotCell.from_urdf_and_srdf(
            urdf_filename=compas_fab.get("robot_library/ur10e_robot/urdf/robot_description.urdf"),
            srdf_filename=compas_fab.get("robot_library/ur10e_robot/robot_description_semantic.srdf"),
            local_package_mesh_folder="robot_library/ur10e_robot" if load_geometry else None,
        )
        robot_cell_state = RobotCellState.from_robot_cell(robot_cell)

        return robot_cell, robot_cell_state

    @classmethod
    def abb_irb4600_40_255(cls, load_geometry=True):
        # type: (Optional[bool]) -> Tuple[RobotCell, RobotCellState]
        """Returns a ABB irb4600-40/2.55 robot.

        The returned :class:`compas_fab.robots.Robot` object contains the robot model and semantics.

        Parameters
        ----------
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the robot.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            Newly created instance of the robot.
        """

        robot_cell = RobotCell.from_urdf_and_srdf(
            urdf_filename=compas_fab.get("robot_library/abb_irb4600_40_255/urdf/robot_description.urdf"),
            srdf_filename=compas_fab.get("robot_library/abb_irb4600_40_255/robot_description_semantic.srdf"),
            local_package_mesh_folder="robot_library/abb_irb4600_40_255" if load_geometry else None,
        )
        robot_cell_state = RobotCellState.from_robot_cell(robot_cell)

        return robot_cell, robot_cell_state

    @classmethod
    def abb_irb120_3_58(cls, load_geometry=True):
        # type: (Optional[bool]) -> Tuple[RobotCell, RobotCellState]
        """Returns a ABB irb120-3/58 robot.

        The returned :class:`compas_fab.robots.Robot` object contains the robot model and semantics.

        Parameters
        ----------
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the robot.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            Newly created instance of the robot.
        """

        robot_cell = RobotCell.from_urdf_and_srdf(
            urdf_filename=compas_fab.get("robot_library/abb_irb120_3_58/urdf/robot_description.urdf"),
            srdf_filename=compas_fab.get("robot_library/abb_irb120_3_58/robot_description_semantic.srdf"),
            local_package_mesh_folder="robot_library/abb_irb120_3_58" if load_geometry else None,
        )
        robot_cell_state = RobotCellState.from_robot_cell(robot_cell)

        return robot_cell, robot_cell_state

    @classmethod
    def panda(cls, load_geometry=True):
        # type: (Optional[bool]) -> Tuple[RobotCell, RobotCellState]
        """Returns a Panda robot.

        The returned :class:`compas_fab.robots.Robot` object contains the robot model and semantics.

        Parameters
        ----------
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the robot.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            Newly created instance of the robot.
        """

        robot_cell = RobotCell.from_urdf_and_srdf(
            urdf_filename=compas_fab.get("robot_library/panda/urdf/robot_description.urdf"),
            srdf_filename=compas_fab.get("robot_library/panda/robot_description_semantic.srdf"),
            local_package_mesh_folder="robot_library/panda" if load_geometry else None,
        )

        # Remove the links with '_sc' suffix, which are collision objects used for
        # Gazebo simulation. They are not needed for PyBullet and obscure the gui.
        # They are also not helpful in other viewers.
        # See https://frankaemika.github.io/docs/franka_ros.html for more details.
        robot_model = robot_cell.robot_model
        for link in robot_model.links:
            if link.name.endswith("_sc"):
                robot_model.remove_link(link.name)
                robot_model.remove_joint(link.parent_joint.name)

        robot_cell_state = RobotCellState.from_robot_cell(robot_cell)

        return robot_cell, robot_cell_state

    # ---------------------------------------------------------------------
    # Robot Cells with Tools and Rigid Bodies
    # ---------------------------------------------------------------------

    @classmethod
    def ur5_cone_tool(cls, load_geometry=True):
        # type: (Optional[bool]) -> Tuple[RobotCell, RobotCellState]
        """Create and return the UR5 robot with a cone tool attached. A floor is also included.

        See :meth:`compas_fab.robots.RobotCellLibrary.ur5` and :meth:`compas_fab.robots.ToolLibrary.cone`
        for details on the robot and tool.

        Parameters
        ----------
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the robot and tool geometry are loaded.
            `False` can be used to speed up the creation of the robot cell,
            but without geometry, the robot cell cannot be visualized and backend planners
            cannot perform collision checking during planning.

        Returns
        -------
        Tuple[:class:`compas_fab.robots.RobotCell`, :class:`compas_fab.robots.RobotCellState`]
            Newly created instance of the robot cell and robot cell state.
        """
        # ---------------------------------------------------------------------
        # Load Robot and create RobotCell
        # ---------------------------------------------------------------------
        robot_cell, robot_cell_state = RobotCellLibrary.ur5(load_geometry=load_geometry)

        # ---------------------------------------------------------------------
        # Load Tools
        # ---------------------------------------------------------------------

        cone_radius = 0.02
        cone_length = 0.1
        cone = ToolLibrary.cone(load_geometry=load_geometry, radius=cone_radius, length=cone_length)
        robot_cell.tool_models["cone"] = cone

        # ---------------------------------------------------------------------
        # Load Rigid Bodies
        # ---------------------------------------------------------------------

        # Static Floor as Collision Geometry
        floor_mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
        robot_cell.rigid_body_models["floor"] = RigidBody.from_mesh(floor_mesh)

        # ------------------------------------------------------------------------
        # Re-Create RobotCellState after modifying the RobotCell
        # ------------------------------------------------------------------------
        robot_cell_state = RobotCellState.from_robot_cell(robot_cell)

        # Attach the tool to the robot's main group
        touch_links = ["wrist_3_link"]
        # UR5 has the last planning link as 'tool0' not 'flange', therefore the cone tool
        # that is REP 199 compliant is attached with the following rotation to match.
        attachment_frame = Frame([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
        robot_cell_state.set_tool_attached_to_group(
            "cone", robot_cell.main_group_name, attachment_frame=attachment_frame, touch_links=touch_links
        )

        # ------------------------------------------------------------------------
        # Static Rigid Body Touch Links
        # ------------------------------------------------------------------------

        # The floor is not attached to the robot, but it is allowed to touch the robot's base link.
        robot_cell_state.rigid_body_states["floor"].touch_links = ["base_link_inertia"]

        return robot_cell, robot_cell_state

    @classmethod
    def abb_irb4600_40_255_gripper_one_beam(cls, load_geometry=True):
        # type: (Optional[bool]) -> Tuple[RobotCell, RobotCellState]
        """Create and return the ABB irb4600-40-255 robot with a gripper tool attached.
        One beam (a RigidBody) is included and is attached to the gripper.
        A floor is also included.

        See :meth:`compas_fab.robots.RobotCellLibrary.abb_irb4600_40_255` and :meth:`compas_fab.robots.ToolLibrary.static_gripper`
        for details on the robot and tool.

        Parameters
        ----------
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the robot and tool geometry are loaded.
            `False` can be used to speed up the creation of the robot cell,
            but without geometry, the robot cell cannot be visualized and backend planners
            cannot perform collision checking during planning.

        Returns
        -------
        Tuple[:class:`compas_fab.robots.RobotCell`, :class:`compas_fab.robots.RobotCellState`]
            Newly created instance of the robot cell and robot cell state.
        """
        # ---------------------------------------------------------------------
        # Load Robot and create RobotCell
        # ---------------------------------------------------------------------
        robot_cell, robot_cell_state = RobotCellLibrary.abb_irb4600_40_255(load_geometry=load_geometry)

        # ---------------------------------------------------------------------
        # Load Tools
        # ---------------------------------------------------------------------

        gripper = ToolLibrary.static_gripper(load_geometry=load_geometry)
        robot_cell.tool_models["gripper"] = gripper

        # ---------------------------------------------------------------------
        # Load Rigid Bodies
        # ---------------------------------------------------------------------

        # Z axis is the length of the beam, X axis points away from the robot
        beam_length = 1.0
        beam = Box.from_corner_corner_height(
            [0.0, -0.1, -beam_length * 0.5], [0.2, 0.1, -beam_length * 0.5], beam_length
        )
        beam_mesh = Mesh.from_shape(beam)
        robot_cell.rigid_body_models["beam"] = RigidBody.from_mesh(beam_mesh)

        # Static Floor as Collision Geometry
        floor_mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
        robot_cell.rigid_body_models["floor"] = RigidBody.from_mesh(floor_mesh)

        # ------------------------------------------------------------------------
        # Re-Create RobotCellState after modifying the RobotCell
        # ------------------------------------------------------------------------
        robot_cell_state = RobotCellState.from_robot_cell(robot_cell)

        # Attach the tool to the robot's main group
        attachment_frame = Frame([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
        robot_cell_state.set_tool_attached_to_group("gripper", robot_cell.main_group_name, attachment_frame)
        # Note: There is a rotation to match the gripper's orientation because the last link in the abb robot
        # does not follow the REP 199 convention.

        # Attach the beam to the gripper
        robot_cell_state.set_rigid_body_attached_to_tool("beam", "gripper")

        return robot_cell, robot_cell_state

    @classmethod
    def ur10e_gripper_one_beam(cls, load_geometry=True):
        # type: (Optional[bool]) -> Tuple[RobotCell, RobotCellState]
        """Create and return the ur10e robot with a gripper tool attached.
        One beam (a RigidBody) is included and is attached to the gripper.
        A floor is also included.

        See :meth:`compas_fab.robots.RobotCellLibrary.ur10e` and :meth:`compas_fab.robots.ToolLibrary.static_gripper_small`
        for details on the robot and tool.

        Parameters
        ----------
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the robot and tool geometry are loaded.
            `False` can be used to speed up the creation of the robot cell,
            but without geometry, the robot cell cannot be visualized and backend planners
            cannot perform collision checking during planning.

        Returns
        -------
        Tuple[:class:`compas_fab.robots.RobotCell`, :class:`compas_fab.robots.RobotCellState`]
            Newly created instance of the robot cell and robot cell state.
        """
        # ---------------------------------------------------------------------
        # Load Robot and create RobotCell
        # ---------------------------------------------------------------------
        robot_cell, robot_cell_state = RobotCellLibrary.ur10e(load_geometry=load_geometry)

        # ---------------------------------------------------------------------
        # Load Tools
        # ---------------------------------------------------------------------

        gripper = ToolLibrary.static_gripper_small(load_geometry=load_geometry)
        robot_cell.tool_models["gripper"] = gripper

        # ---------------------------------------------------------------------
        # Load Rigid Bodies
        # ---------------------------------------------------------------------

        # Z axis is the length of the beam, X axis points away from the robot
        beam_length = 0.4
        beam = Box.from_corner_corner_height(
            [0.0, -0.05, -beam_length * 0.5], [0.1, 0.05, -beam_length * 0.5], beam_length
        )
        beam_mesh = Mesh.from_shape(beam)
        robot_cell.rigid_body_models["beam"] = RigidBody.from_mesh(beam_mesh)

        # Static Floor as Collision Geometry
        floor_mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
        robot_cell.rigid_body_models["floor"] = RigidBody.from_mesh(floor_mesh)

        # ------------------------------------------------------------------------
        # Re-Create RobotCellState after modifying the RobotCell
        # ------------------------------------------------------------------------
        robot_cell_state = RobotCellState.from_robot_cell(robot_cell)

        # ------------------------------------------------------------------------
        # Tool Attachment
        # ------------------------------------------------------------------------

        # Attach the tool to the robot's main group
        attachment_frame = Frame([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])

        # Gripper is allowed to touch the last link of the robot.
        # However, do not use the following line to get the end effector link name,
        # because it is not guaranteed to be the last link that has geometry in the robot chain.
        # ee_link_name = robot.get_end_effector_link_name(robot_cell.main_group_name)

        # Instead, check the robot model and hard code the actual link name.
        touch_links = ["wrist_3_link"]
        # For UR10e, the last logical link is `tool0` (from robot.get_end_effector_link_name)
        # However the last link with geometry attached is `wrist_3_link`.

        robot_cell_state.set_tool_attached_to_group(
            "gripper", robot_cell.main_group_name, attachment_frame, touch_links
        )
        # Note: There is a rotation to match the gripper's orientation because the last link in the abb robot
        # does not follow the REP 199 convention.

        # ------------------------------------------------------------------------
        # Workpiece Attachment
        # ------------------------------------------------------------------------

        # Attach the beam to the gripper
        robot_cell_state.set_rigid_body_attached_to_tool("beam", "gripper")

        # ------------------------------------------------------------------------
        # Static Rigid Body Touch Links
        # ------------------------------------------------------------------------

        # The floor is not attached to the robot, but it is allowed to touch the robot's base link.
        robot_cell_state.rigid_body_states["floor"].touch_links = ["base_link_inertia"]

        return robot_cell, robot_cell_state

    @classmethod
    def abb_irb4600_40_255_printing_tool(cls, load_geometry=True):
        # type: (Optional[bool]) -> Tuple[RobotCell, RobotCellState]
        """Create and return the ABB irb4600-40-255 robot with a printing tool attached.
        A floor is also included.

        See :meth:`compas_fab.robots.RobotCellLibrary.abb_irb4600_40_255` and :meth:`compas_fab.robots.ToolLibrary.printing_tool`
        for details on the robot and tool.

        Parameters
        ----------
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the robot and tool geometry are loaded.
            `False` can be used to speed up the creation of the robot cell,
            but without geometry, the robot cell cannot be visualized and backend planners
            cannot perform collision checking during planning.

        Returns
        -------
        Tuple[:class:`compas_fab.robots.RobotCell`, :class:`compas_fab.robots.RobotCellState`]
            Newly created instance of the robot cell and robot cell state.
        """
        # ---------------------------------------------------------------------
        # Load Robot and create RobotCell
        # ---------------------------------------------------------------------
        robot_cell, robot_cell_state = RobotCellLibrary.abb_irb4600_40_255(load_geometry=load_geometry)

        # ---------------------------------------------------------------------
        # Load Tools
        # ---------------------------------------------------------------------

        printing_tool = ToolLibrary.printing_tool(load_geometry=load_geometry, tool_size=0.5)
        robot_cell.tool_models["printing_tool"] = printing_tool

        # ---------------------------------------------------------------------
        # Load Rigid Bodies
        # ---------------------------------------------------------------------

        # Static Floor as Collision Geometry
        floor_mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
        robot_cell.rigid_body_models["floor"] = RigidBody.from_mesh(floor_mesh)

        # ------------------------------------------------------------------------
        # Re-Create RobotCellState after modifying the RobotCell
        # ------------------------------------------------------------------------
        robot_cell_state = RobotCellState.from_robot_cell(robot_cell)

        # ------------------------------------------------------------------------
        # Tool Attachment
        # ------------------------------------------------------------------------

        # Attach the tool to the robot's main group
        attachment_frame = Frame([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
        touch_links = ["link_6"]
        robot_cell_state.set_tool_attached_to_group(
            "printing_tool", robot_cell.main_group_name, attachment_frame, touch_links=touch_links
        )

        # ------------------------------------------------------------------------
        # Static Rigid Body Touch Links
        # ------------------------------------------------------------------------

        # The floor is not attached to the robot, but it is allowed to touch the robot's base link.
        robot_cell_state.rigid_body_states["floor"].touch_links = ["base_link"]

        return robot_cell, robot_cell_state


if __name__ == "__main__":

    # ----------------------------
    # Visualize Tool with compas_viewer
    # ----------------------------

    from compas_robots.viewer.scene.robotmodelobject import RobotModelObject  # noqa: F401
    from compas_viewer import Viewer

    viewer = Viewer()
    viewer.renderer.rendermode = "lighted"

    model = ToolLibrary.static_gripper(load_geometry=True)
    robot_object = viewer.scene.add(model, show_lines=False)  # type: RobotModelObject

    marker = RigidBodyLibrary.target_marker(size=0.5)
    marker_object = viewer.scene.add(marker.visual_meshes[0], show_lines=False)

    viewer.show()

    # ----------------------------
    # Visualize with pybullet gui
    # ----------------------------

    # from compas_fab.backends.pybullet import PyBulletClient
    # from compas_fab.backends.pybullet import PyBulletPlanner

    # with PyBulletClient() as client:
    #     planner = PyBulletPlanner(client)
    #     robot_cell, robot_cell_state = RobotCellLibrary.abb_irb4600_40_255_gripper_one_beam(client, load_geometry=True)
    #     planner.set_robot_cell(robot_cell, robot_cell_state)
    #     input("Press Enter to exit...")
