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
    CAD environment (:meth:`draw_geometry`) and the other is one to apply a transformation
    to geometry (:meth:`transform`).

    Attributes
    ----------
    robot : :class:`compas.robots.RobotModel`
        Instance of a robot model.
    """

    def __init__(self, robot):
        super(BaseRobotArtist, self).__init__()
        self.robot = robot
        self.create()
        self.scale_factor = 1.

    def transform(self, geometry, transformation):
        """Transforms a CAD-specific geometry using a **COMPAS** transformation.

        Parameters
        ----------
        geometry : object
            A CAD-specific (i.e. native) geometry object as returned by :meth:`draw_geometry`.
        transformation : `Transformation`
            **COMPAS** transformation to update the geometry object.
        """
        raise NotImplementedError

    def draw_geometry(self, geometry, color=None):
        """Draw a **COMPAS** geometry in the respective CAD environment.

        Note
        ----
        This is an abstract method that needs to be implemented by derived classes.

        Parameters
        ----------
        geometry : :class:`compas.datastructures.Mesh`
            Instance of a **COMPAS** mesh.

        Returns
        -------
        object
            CAD-specific geometry
        """
        raise NotImplementedError

    def create(self, link=None):
        """Recursive function that triggers the drawing of the robot geometry.

        This method delegates the geometry drawing to the :meth:`draw_geometry`
        method. It transforms the geometry based on the saved initial
        transformation from the robot model.

        Parameters
        ----------
        link : :class:`compas.robots.Link`, optional
            Link instance to create. Defaults to the robot model's root.

        Returns
        -------
        None
        """
        if link is None:
            link = self.robot.root

        for item in itertools.chain(link.visual, link.collision):
            if item.geometry.geo:
                color = None
                if hasattr(item, 'get_color'):
                    color = item.get_color()
                # create native geometry
                item.native_geometry = self.draw_geometry(item.geometry.geo, color)
                # transform native geometry based on saved init transform
                self.transform(item.native_geometry, item.init_transformation)
                item.current_transformation = Transformation()

        for child_joint in link.joints:
            self.create(child_joint.child_link)

    def scale(self, factor):
        """Scales the robot geometry by factor (absolute).

        Parameters
        ----------
        factor : float
            The factor to scale the robot with.

        Returns
        -------
        None
        """
        self.robot.scale(factor)  # scale the model

        relative_factor = factor / self.scale_factor  # relative scaling factor
        transformation = Scale([relative_factor, relative_factor, relative_factor])
        self.scale_link(self.robot.root, transformation)
        self.scale_factor = factor

    def scale_link(self, link, transformation):
        """Recursive function to apply the scale transformation on each link.
        """
        for item in itertools.chain(link.visual, link.collision):
            # Some links have only collision geometry, not visual. These meshes
            # have not been loaded.
            if item.native_geometry:
                self.transform(item.native_geometry, transformation)

        for child_joint in link.joints:
            # Recursive call
            self.scale_link(child_joint.child_link, transformation)

    def _apply_transformation_on_transformed_link(self, item, transformation):
        """Applies a transformation on a link that is already transformed.

        Calculates the relative transformation and applies it to the link
        geometry. This is to prevent the recreation of large meshes.

        Parameters
        ----------
        item: :class:`compas.robots.Visual` or :class:`compas.robots.Collision`
            The visual or collidable object of a link.
        transformation: :class:`Transformation`
            The (absolute) transformation to apply onto the link's geometry.

        Returns
        -------
        None
        """
        relative_transformation = transformation * item.current_transformation.inverse()
        self.transform(item.native_geometry, relative_transformation)
        item.current_transformation = transformation

    def update(self, configuration, names, visual=True, collision=True):
        """Triggers the update of the robot geometry.

        Parameters
        ----------
        configuration : :class:`compas_fab.robots.Configuration`
            Instance of the configuration (joint state) to move to.
        names : list of string
            The names of the configurable joints to update.
        visual : bool, optional
            ``True`` if the visual geometry should be also updated, otherwise ``False``.
            Defaults to ``True``.
        collision : bool, optional
            ``True`` if the collision geometry should be also updated, otherwise ``False``.
            Defaults to ``True``.
        """
        positions = configuration.values
        if len(names) != len(positions):
            raise ValueError("len(names): %d is not len(positions) %d" % (len(names), len(positions)))
        joint_state = dict(zip(names, positions))
        transformations = self.robot.compute_transformations(joint_state)
        for j in self.robot.iter_joints():
            link = j.child_link
            for item in link.visual:
                self._apply_transformation_on_transformed_link(item, transformations[j.name])
            if collision:
                for item in link.collision:
                    # some links have only collision geometry, not visual. These meshes have not been loaded.
                    if item.native_geometry:
                        self._apply_transformation_on_transformed_link(item, transformations[j.name])

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
