from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import itertools

from compas.geometry import Frame
from compas.geometry import Transformation
from compas.geometry import Scale

__all__ = [
    'BaseRobotArtist'
]


class BaseRobotArtist(object):
    """Provides common functionality to most robot artist implementations.

    In **COMPAS**, the `artists` are classes that assist with the visualization of
    datastructures and models, in a way that maintains the data separated from the
    specific CAD interfaces, while providing a way to leverage native performance
    of the CAD environment.

    There are two methods that implementors of this base class should provide, one
    is concerned with the actual creation of geometry in the native format of the
    CAD environment (:meth:`draw_mesh`) and the other is one to apply a transformation
    to geometry (:meth:`transform`).

    Attributes
    ----------
    robot : :class:`compas.robots.RobotModel`
        Instance of a robot model.
    """

    def __init__(self, robot):
        super(BaseRobotArtist, self).__init__()
        self.robot = robot
        self.create(robot.root, Transformation())
        self.scale_factor = 1.

    def transform(self, native_mesh, transformation):
        """Transforms a CAD-specific mesh using a **COMPAS** transformation.

        Parameters
        ----------
        native_mesh : object
            A CAD-specific (i.e. native) geometry object as returned by :meth:`draw_mesh`.
        transformation : `Transformation`
            **COMPAS** transformation to update the geometry object.
        """
        raise NotImplementedError

    def draw_mesh(self, mesh, color=None):
        """Draw a **COMPAS** mesh into the CAD environment.

        Note
        ----
        This is an abstract method that needs to be implemented by derived classes.

        Parameters
        ----------
        mesh : :class:`compas.datastructures.Mesh`
            Instance of a **COMPAS** mesh.

        Returns
        -------
        object
            CAD-specific geometry
        """
        raise NotImplementedError

    def create(self, link, parent_transformation):
        """Triggers the drawing of the robot geometry.

        This method delegates the geometry drawing to the :meth:`draw_mesh` method.

        Parameters
        ----------
        link : :class:`compas.robots.Link`
            Link instance to create.
        parent_transformation : :class:`Transformation`
            Parent transformation to apply to the link when creating the structure.

        """
        for item in itertools.chain(link.visual, link.collision):
            if item.geometry.geo:
                color = None
                if hasattr(item, 'get_color'):
                    color = item.get_color()
                item.native_geometry = self.draw_mesh(item.geometry.geo, color)
                self.transform(item.native_geometry, parent_transformation)
            else:
                item.native_geometry = None

        for child_joint in link.joints:
            child_joint.create(parent_transformation)
            # Recursively call creation
            self.create(child_joint.child_link, child_joint.current_transformation)

    def scale(self, factor):
        """Scales the robot geometry by factor (absolute).

        Parameters
        ----------
        factor : float
            The factor to scale the robot with.
        """
        relative_factor = factor / self.scale_factor # relative scaling factor
        self.scale_factor = factor
        transformation = Scale([relative_factor, relative_factor, relative_factor])
        self.scale_links(transformation)

    def scale_links(self, transformation):
        self.scale_link(self.robot.root, transformation)

    def scale_link(self, link, transformation):
        """Recursive function to apply the scale transformation on each link
            geometry.
        """
        relative_factor = transformation[0,0]
        for item in itertools.chain(link.visual, link.collision):
            # some links have only collision geometry, not visual. These meshes
            # have not been loaded.
            if item.native_geometry:
                self.transform(item.native_geometry, transformation)

        for child_joint in link.joints:
            child_joint.scale(relative_factor)
            # Recursive call
            self.scale_link(child_joint.child_link, transformation)

    def update(self, configuration, collision=True, names=[]):
        """Trigger the update of the robot geometry.

        Parameters
        ----------
        configuration : :class:`compas_fab.robots.Configuration`
            Instance of the configuration (joint state) to move to.
        collision : bool, optional
            ``True`` if the collision geometry should be also updated, otherwise ``False``.
            Defaults to ``True``.
        """
        if not len(names):
            names = self.robot.get_configurable_joint_names()
        positions = configuration.values
        self.update_links(names, positions, collision)

    def update_links(self, names, positions, collision=True):
        """Updates the joints and link geometries.

        Parameters
        ----------
        names : list of :obj:`str`
          A list of the joints names that are updated.
        positions : list of :obj:`float`
            A list of the respective joint positions, in radians and meters.
        collision : bool
            ``True`` if the collision geometry should be also updated, otherwise ``False``.
            Defaults to ``True``.
        """
        if len(names) != len(positions):
            raise ValueError("len(names): %d is not len(positions) %d" % (len(names), len(positions)))
        joint_state = dict(zip(names, positions))
        self.update_link(self.robot.root, joint_state, Transformation(), Transformation(), collision)

    def update_link(self, link, joint_state, parent_transformation, reset_transformation, collision=True):
        """Recursive function to apply the transformations given by the joint state.

        The parameter ``joint_state`` is given as absolute values,
        so it is necessary to reset the current transformation.

        Parameters
        ----------
        link : :class:`compas.robots.Link`
            Link instance to update.
        joint_state : dict
            A dictionary with the joint names as keys and values
            in radians and meters (depending on the joint type).
        parent_transformation : :class:`Transformation`
            The transfomation of the parent joint.
        collision : bool
            ``True`` if the collision geometry should be also updated, otherwise ``False``.
            Defaults to ``True``.
        """

        relative_transformation = parent_transformation * reset_transformation

        for item in link.visual:
            self.transform(item.native_geometry, relative_transformation)

        if collision:
            for item in link.collision:
                # some links have only collision geometry, not visual. These meshes have not been loaded.
                if item.native_geometry:
                    self.transform(item.native_geometry, relative_transformation)

        for child_joint in link.joints:
            reset_transformation = child_joint.reset_transformation
            # 1. Reset child joint transformation
            child_joint.reset_transform()
            # 2. Calculate transformation for next joints in the chain
            if child_joint.name in joint_state.keys():
                position = joint_state[child_joint.name]
                transformation = child_joint.calculate_transformation(position)
                transformation = parent_transformation * transformation
                child_joint.position = position
            else:
                transformation = parent_transformation

            # 3. Apply on joint
            child_joint.transform(transformation)

            # 4. Apply function to all children in the chain
            self.update_link(child_joint.child_link, joint_state, transformation, reset_transformation, collision)

    def draw_visual(self):
        """Draws all visual geometry of the robot."""
        for link in self.robot.iter_links():
            for item in link.visual:
                yield item.native_geometry

    def draw_collision(self):
        """Draws all collision geometry of the robot."""
        for link in self.robot.iter_links():
            for item in link.collision:
                if item.native_geometry:
                    yield item.native_geometry
