from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


from compas import IPY
from compas.data import Data
from compas.geometry import Frame
from compas.geometry import Transformation
from compas.tolerance import TOL
from compas_robots import Configuration
from compas_robots import RobotModel
from compas_robots.model import Joint


from compas_fab.robots.constraints import Constraint

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Any  # noqa: F401
        from typing import Dict  # noqa: F401
        from typing import Generator  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Optional  # noqa: F401
        from typing import Tuple  # noqa: F401

        from compas.geometry import Vector  # noqa: F401
        from compas_robots.model import Joint  # noqa: F401, F811
        from compas_robots.model import Link  # noqa: F401
        from compas_robots.model import Material  # noqa: F401
        from compas_robots.scene import BaseRobotModelObject  # noqa: F401

        from compas_fab.robots import JointTrajectory  # noqa: F401
        from compas_fab.robots import RobotSemantics  # noqa: F401
        from compas_fab.robots import Target  # noqa: F401
        from compas_fab.robots import Tool  # noqa: F401
        from compas_fab.robots import Waypoints  # noqa: F401


__all__ = [
    "Robot",
]


class Robot(Data):
    """Represents a robot.

    This class binds together several building blocks, such as the robot's
    descriptive model and its semantic information
    into a cohesive programmable interface. This representation builds
    upon the model described in the class :class:`compas_robots.RobotModel` of
    the **COMPAS** framework.

    Parameters
    ----------
    model : :class:`compas_robots.RobotModel`
        The robot model that describes robot kinematics, typically comes from an URDF structure.
    scene_object : :class:`compas_robots.scene.BaseRobotModelObject`
        Instance of the scene object used to visualize the robot model. Defaults to ``None``.
    semantics : :class:`~compas_fab.robots.RobotSemantics`
        The semantic model of the robot. Defaults to ``None``.

    Attributes
    ----------
    attributes : :obj:`dict`
        Named attributes related to the robot instance.
    attached_tools : :obj:`dict` of [:obj:`str`, :class:`~compas_fab.robots.Tool`], read-only
    attached_tool : :class:`~compas_fab.robots.Tool`, read-only
    group_names : :obj:`list` of :obj:`str`, read-only
    group_states : :obj:`dict` of :obj:`dict`, read-only
    scene_object : :class:`compas_robots.scene.BaseRobotModelObject`
    main_group_name : :obj:`str`, read-only
    model : :class:`compas_robots.RobotModel`
        The robot model that describes robot kinematics, typically comes from an URDF structure.
    name : :obj:`str`, read-only
        Name of the robot, as defined by its model.
    root_name : :obj:`str`, read-only
    semantics : :class:`~compas_fab.robots.RobotSemantics`
        The semantic model of the robot. Can be ``None`` if not loaded.
    scale_factor : :obj:`float`

    """

    # NOTE: If the attribute function has a docstring, the first sentence will be used automatically in the class attribute's.
    #       However, the rest of the docstring, after the first period symbol will be ignored.
    #       It is futile to add examples to the attribute docstring, as they will not be rendered in the documentation.

    def __init__(self, model=None, scene_object=None, semantics=None):
        # type: (RobotModel, Optional[BaseRobotModelObject], Optional[RobotSemantics]) -> Robot
        super(Robot, self).__init__()
        # These attributes have to be initiated first,
        # because they are used in the setters of the other attributes
        self._scale_factor = 1.0
        self._attached_tools = {}  # { planning_group_name: robots.tool.Tool }
        self._current_ik = {"request_id": None, "solutions": None}

        self.model = model
        self.scene_object = scene_object
        self.semantics = semantics
        self.attributes = {}

    @property
    def scene_object(self):
        # type: () -> BaseRobotModelObject | None
        """Scene object used to visualize robot model."""
        return self._scene_object

    @scene_object.setter
    def scene_object(self, value):
        self._scene_object = value
        if value is None:
            return
        # TODO: The whole SceneObject needs some refactoring on how to handle scale
        if len(self.model.joints) > 0 and len(self.model.links) > 0:
            self.scale(self._scale_factor)
            for tool in self.attached_tools.values():
                self.scene_object.attach_tool_model(tool.tool_model)

    # ==========================================================================
    # configurations
    # ==========================================================================

    # ==========================================================================
    # transformations, coordinate frames
    # ==========================================================================

    # ==========================================================================
    # checks
    # ==========================================================================

    # ==========================================================================
    # services
    # ==========================================================================

    def transformed_frames(self, configuration, group=None):
        # type: (Configuration, Optional[str]) -> List[Frame]
        """Get the robot's transformed frames.

        Parameters
        ----------
        configuration : :class:`compas_robots.Configuration`
            Configuration to compute transformed frames for.
        group : :obj:`str`, optional
            The planning group used for the calculation. Defaults to the robot's
            main planning group.

        Returns
        -------
        :obj:`list` of :class:`compas.geometry.Frame`
            Transformed frames.
        """
        if not len(configuration.joint_names):
            configuration.joint_names = self.get_configurable_joint_names(group)
        return self.model.transformed_frames(configuration)

    def transformed_axes(self, configuration, group=None):
        # type: (Configuration, Optional[str]) -> List[Vector]
        """Get the robot's transformed axes.

        Parameters
        ----------
        configuration : :class:`compas_robots.Configuration`
            Configuration to compute transformed axes for.
        group : :obj:`str`, optional
            The planning group used for the calculation. Defaults to the robot's
            main planning group.

        Returns
        -------
        :obj:`list` of :class:`compas.geometry.Vector`
            Transformed axes.
        """
        if not len(configuration.joint_names):
            configuration.joint_names = self.get_configurable_joint_names(group)
        return self.model.transformed_axes(configuration)

    # ==========================================================================
    # drawing
    # ==========================================================================

    def update(self, configuration, group=None, visual=True, collision=True):
        # type: (Configuration, Optional[str], Optional[bool], Optional[bool]) -> None
        """Update the robot's geometry.

        Parameters
        ----------
        configuration : :class:`compas_robots.Configuration`
            Instance of the configuration (joint state) to move to.
        group : :obj:`str`, optional
            The name of the group to plan for. Defaults to the robot's main
            planning group.
        visual : :obj:`bool`, optional
            ``True`` if the visual geometry should be also updated, otherwise ``False``.
            Defaults to ``True``.
        collision : :obj:`bool`, optional
            ``True`` if the collision geometry should be also updated, otherwise ``False``.
            Defaults to ``True``.
        """
        group = group or self.main_group_name if self.semantics else None

        if not len(configuration.joint_names):
            configuration.joint_names = self.get_configurable_joint_names(group)

        self.scene_object.update(configuration, visual, collision)

    def draw_visual(self):
        # type: () -> None
        """Draw the robot's visual geometry using the defined :attr:`Robot.scene_object`."""
        return self.scene_object.draw_visual()

    def draw_collision(self):
        # type: () -> None
        """Draw the robot's collision geometry using the defined :attr:`Robot.scene_object`."""
        return self.scene_object.draw_collision()

    def draw(self):
        # type: () -> None
        """Alias of :meth:`draw_visual`."""
        return self.draw_visual()

    # TODO: add scene_object.draw_attached_tool
    # def draw_attached_tool(self):
    #     """Draw the attached tool using the defined :attr:`Robot.scene_object`."""
    #     if self.scene_object and self.attached_tool:
    #         return self.scene_object.draw_attached_tool()
