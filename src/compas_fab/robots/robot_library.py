from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas
import compas_fab

from compas_fab.robots import Robot
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RigidBody
from compas_robots import ToolModel
from compas.geometry import Frame
from compas.geometry import Cone
from compas.geometry import Box
from compas.datastructures import Mesh


if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Optional  # noqa: F401
        from typing import Dict
        from typing import List
        from typing import Tuple
        from compas_fab.backends.interfaces import ClientInterface  # noqa: F401


__all__ = [
    "RobotLibrary",
    "ToolLibrary",
    "RobotCellLibrary",
    "RigidBodyLibrary",
]


class RobotLibrary(object):
    """A collection of built-in robots that can be used for testing and demonstration.
    The :class:`compas_fab.robots.Robot` objects created by the factory methods
    can be used to write examples, so that the example code can stay short.

    The robots are loaded from URDF, SRDF and local mesh files.
    The resulting robot object
    contains the robot model, semantics, visual and collision meshes for the links.

    Examples
    --------

    >>> from compas_fab.robots import RobotLibrary
    >>> robot = RobotLibrary.ur5()
    >>> robot.name
    'ur5_robot'
    """

    @classmethod
    def rfl(cls, client=None, load_geometry=True):
        # type: (Optional[ClientInterface], Optional[bool]) -> Robot
        """Create and return the RFL robot with 4 ABB irb 4600 and twin-gantry setup.

        The returned :class:`compas_fab.robots.Robot` object contains the robot model and semantics.

        Parameters
        ----------
        client: :class:`compas_fab.backends.interfaces.ClientInterface`, optional
            Backend client. Default is `None`.
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the robot.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            Newly created instance of the robot.
        """

        robot = Robot.from_urdf(
            urdf_filename=compas_fab.get("robot_library/rfl/urdf/robot_description.urdf"),
            srdf_filename=compas_fab.get("robot_library/rfl/robot_description_semantic.srdf"),
            local_package_mesh_folder="robot_library/rfl" if load_geometry else None,
            client=client,
        )

        return robot

    @classmethod
    def ur5(cls, client=None, load_geometry=True):
        # type: (Optional[ClientInterface], Optional[bool]) -> Robot
        """Returns a UR5 robot.

        The returned :class:`compas_fab.robots.Robot` object contains the robot model and semantics.

        The main planning group of the robot is named 'manipulator'.
        The first and last link on the 'manipulator' group is named 'base_link' and 'tool0'.

        Parameters
        ----------
        client: :class:`compas_fab.backends.interfaces.ClientInterface`, optional
            Backend client. Default is `None`.
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the robot.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            Newly created instance of the robot.
        """

        robot = Robot.from_urdf(
            urdf_filename=compas_fab.get("robot_library/ur5_robot/urdf/robot_description.urdf"),
            srdf_filename=compas_fab.get("robot_library/ur5_robot/robot_description_semantic.srdf"),
            local_package_mesh_folder="robot_library/ur5_robot" if load_geometry else None,
            client=client,
        )

        return robot

    @classmethod
    def ur10e(cls, client=None, load_geometry=True):
        # type: (Optional[ClientInterface], Optional[bool]) -> Robot
        """Returns a UR10e robot.

        The returned :class:`compas_fab.robots.Robot` object contains the robot model and semantics.

        Parameters
        ----------
        client: :class:`compas_fab.backends.interfaces.ClientInterface`, optional
            Backend client. Default is `None`.
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the robot.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            Newly created instance of the robot.
        """

        robot = Robot.from_urdf(
            urdf_filename=compas_fab.get("robot_library/ur10e_robot/urdf/robot_description.urdf"),
            srdf_filename=compas_fab.get("robot_library/ur10e_robot/robot_description_semantic.srdf"),
            local_package_mesh_folder="robot_library/ur10e_robot" if load_geometry else None,
            client=client,
        )

        return robot

    @classmethod
    def abb_irb4600_40_255(cls, client=None, load_geometry=True):
        # type: (Optional[ClientInterface], Optional[bool]) -> Robot
        """Returns a ABB irb4600-40/2.55 robot.

        The returned :class:`compas_fab.robots.Robot` object contains the robot model and semantics.

        Parameters
        ----------
        client: :class:`compas_fab.backends.interfaces.ClientInterface`, optional
            Backend client. Default is `None`.
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the robot.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            Newly created instance of the robot.
        """

        robot = Robot.from_urdf(
            urdf_filename=compas_fab.get("robot_library/abb_irb4600_40_255/urdf/robot_description.urdf"),
            srdf_filename=compas_fab.get("robot_library/abb_irb4600_40_255/robot_description_semantic.srdf"),
            local_package_mesh_folder="robot_library/abb_irb4600_40_255" if load_geometry else None,
            client=client,
        )

        return robot

    @classmethod
    def abb_irb120_3_58(cls, client=None, load_geometry=True):
        # type: (Optional[ClientInterface], Optional[bool]) -> Robot
        """Returns a ABB irb120-3/58 robot.

        The returned :class:`compas_fab.robots.Robot` object contains the robot model and semantics.

        Parameters
        ----------
        client: :class:`compas_fab.backends.interfaces.ClientInterface`, optional
            Backend client. Default is `None`.
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the robot.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            Newly created instance of the robot.
        """

        robot = Robot.from_urdf(
            urdf_filename=compas_fab.get("robot_library/abb_irb120_3_58/urdf/robot_description.urdf"),
            srdf_filename=compas_fab.get("robot_library/abb_irb120_3_58/robot_description_semantic.srdf"),
            local_package_mesh_folder="robot_library/abb_irb120_3_58" if load_geometry else None,
            client=client,
        )

        return robot

    @classmethod
    def panda(cls, client=None, load_geometry=True):
        # type: (Optional[ClientInterface], Optional[bool]) -> Robot
        """Returns a Panda robot.

        The returned :class:`compas_fab.robots.Robot` object contains the robot model and semantics.

        Parameters
        ----------
        client: :class:`compas_fab.backends.interfaces.ClientInterface`, optional
            Backend client. Default is `None`.
        load_geometry: :obj:`bool`, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the robot.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            Newly created instance of the robot.
        """

        robot = Robot.from_urdf(
            urdf_filename=compas_fab.get("robot_library/panda/urdf/robot_description.urdf"),
            srdf_filename=compas_fab.get("robot_library/panda/robot_description_semantic.srdf"),
            local_package_mesh_folder="robot_library/panda" if load_geometry else None,
            client=client,
        )

        # Remove the links with '_sc' suffix, which are collision objects used for
        # Gazebo simulation. They are not needed for PyBullet and obscure the gui.
        # They are also not helpful in other viewers.
        # See https://frankaemika.github.io/docs/franka_ros.html for more details.
        for link in robot.model.links:
            if link.name.endswith("_sc"):
                robot.model.remove_link(link.name)
                robot.model.remove_joint(link.parent_joint.name)

        return robot


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

        if load_geometry:
            meshes = []
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

        if load_geometry:
            meshes = []
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


class RobotCellLibrary(object):
    """A collection of built-in robot cells that can be used for testing and demonstrations.
    The :class:`compas_fab.robots.RobotCell` and :class:`compas_fab.robots.RobotCellState`
    objects created by the factory methods can be used to write examples,
    so that the example code can stay short.

    The Robot object in the RobotCell are loaded by RobotLibrary.
    It contains the robot model, semantics, visual and collision meshes for the links.
    The Tool(s) are loaded by ToolLibrary. It contains the tool model, semantics, visual and collision meshes.
    Other collision objects such as robot backpacks, floors, tables, workpieces, etc. may also be included
    depending on the specific robot cell.

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

    @classmethod
    def ur5_cone_tool(cls, client=None, load_geometry=True):
        # type: (Optional[ClientInterface], Optional[bool]) -> Tuple[RobotCell, RobotCellState]
        """Create and return the UR5 robot with a cone tool attached. A floor is also included.

        See :meth:`compas_fab.robots.RobotLibrary.ur5` and :meth:`compas_fab.robots.ToolLibrary.cone`
        for details on the robot and tool.

        Parameters
        ----------
        client: :class:`compas_fab.backends.interfaces.ClientInterface`, optional
            Backend client. Default is `None`.
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
        robot = RobotLibrary.ur5(client, load_geometry=load_geometry)
        robot_cell = RobotCell(robot)

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
        # Create RobotCellState
        # ------------------------------------------------------------------------
        robot_cell_state = RobotCellState.from_robot_cell(robot_cell)

        # Attach the tool to the robot's main group
        touch_links = ["wrist_3_link"]
        # UR5 has the last planning link as 'tool0' not 'flange', therefore the cone tool
        # that is REP 199 compliant is attached with the following rotation to match.
        attachment_frame = Frame([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
        robot_cell_state.set_tool_attached_to_group(
            "cone", robot.main_group_name, attachment_frame=attachment_frame, touch_links=touch_links
        )

        # ------------------------------------------------------------------------
        # Static Rigid Body Touch Links
        # ------------------------------------------------------------------------

        # The floor is not attached to the robot, but it is allowed to touch the robot's base link.
        robot_cell_state.rigid_body_states["floor"].touch_links = ["base_link_inertia"]

        return robot_cell, robot_cell_state

    @classmethod
    def abb_irb4600_40_255_gripper_one_beam(cls, client=None, load_geometry=True):
        # type: (Optional[ClientInterface], Optional[bool]) -> Tuple[RobotCell, RobotCellState]
        """Create and return the ABB irb4600-40-255 robot with a gripper tool attached.
        One beam (a RigidBody) is included and is attached to the gripper.
        A floor is also included.

        See :meth:`compas_fab.robots.RobotLibrary.abb_irb4600_40_255` and :meth:`compas_fab.robots.ToolLibrary.static_gripper`
        for details on the robot and tool.

        Parameters
        ----------
        client: :class:`compas_fab.backends.interfaces.ClientInterface`, optional
            Backend client. Default is `None`.
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
        robot = RobotLibrary.abb_irb4600_40_255(client, load_geometry=load_geometry)
        robot_cell = RobotCell(robot)

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
        # Create RobotCellState
        # ------------------------------------------------------------------------
        robot_cell_state = RobotCellState.from_robot_cell(robot_cell)

        # Attach the tool to the robot's main group
        attachment_frame = Frame([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
        robot_cell_state.set_tool_attached_to_group("gripper", robot.main_group_name, attachment_frame)
        # Note: There is a rotation to match the gripper's orientation because the last link in the abb robot
        # does not follow the REP 199 convention.

        # Attach the beam to the gripper
        robot_cell_state.set_rigid_body_attached_to_tool("beam", "gripper")

        return robot_cell, robot_cell_state

    @classmethod
    def ur10e_gripper_one_beam(cls, client=None, load_geometry=True):
        # type: (Optional[ClientInterface], Optional[bool]) -> Tuple[RobotCell, RobotCellState]
        """Create and return the ur10e robot with a gripper tool attached.
        One beam (a RigidBody) is included and is attached to the gripper.
        A floor is also included.

        See :meth:`compas_fab.robots.RobotLibrary.ur10e` and :meth:`compas_fab.robots.ToolLibrary.static_gripper_small`
        for details on the robot and tool.

        Parameters
        ----------
        client: :class:`compas_fab.backends.interfaces.ClientInterface`, optional
            Backend client. Default is `None`.
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
        robot = RobotLibrary.ur10e(client, load_geometry=load_geometry)
        robot_cell = RobotCell(robot)

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
        # Create RobotCellState
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
        # ee_link_name = robot.get_end_effector_link_name(robot.main_group_name)

        # Instead, check the robot model and hard code the actual link name.
        touch_links = ["wrist_3_link"]
        # For UR10e, the last logical link is `tool0` (from robot.get_end_effector_link_name)
        # However the last link with geometry attached is `wrist_3_link`.

        robot_cell_state.set_tool_attached_to_group("gripper", robot.main_group_name, attachment_frame, touch_links)
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


if __name__ == "__main__":
    # robot = RobotLibrary.rfl(load_geometry=True)
    # robot.info()

    # robot = RobotLibrary.ur5(load_geometry=True)
    # robot.info()

    # robot_cell, robot_cell_state = RobotCellLibrary.ur5_cone_tool(load_geometry=True)
    # robot_cell.robot.info()
    # robot_cell_state.get_attached_tool_id(robot_cell.robot.main_group_name)

    # ----------------------------
    # Visualize Tool with compas_viewer
    # ----------------------------

    from compas_viewer import Viewer
    from compas_robots.viewer.scene.robotmodelobject import RobotModelObject

    viewer = Viewer()
    viewer.renderer.rendermode = "lighted"

    # model = robot.model
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
