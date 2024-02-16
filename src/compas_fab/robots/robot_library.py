"""This module is mostly for example purposes.

It needs to be explicitely imported, since it is not a fundamental part
of the compas_fab.robots package at all.
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_robots import RobotModel
from compas_robots.resources import LocalPackageMeshLoader

import compas_fab
from .robot import Robot as RobotClass
from .semantics import RobotSemantics

__all__ = [
    "RobotLibrary",
]


class RobotLibrary:
    """A collection of built-in robots that can be used for testing and example purposes.
    This method serves mainly as help for writing examples, so that the code can stay short.

    The robots are loaded from URDF, SRDF and mesh files located locally
    in the folder `src/compas_fab/data/robot_library`

    Examples
    --------

    >>> from compas_fab.robots import RobotLibrary
    >>> robot = RobotLibrary.ur5()
    >>> robot.name
    'ur5'
    """

    @classmethod
    def _load_library_model(
        cls, urdf_filename, srdf_filename, local_package_mesh_folder, client=None, load_geometry=True
    ):
        """Convenience method for loading robot from local cache directory."""

        model = RobotModel.from_urdf_file(urdf_filename)
        semantics = RobotSemantics.from_srdf_file(srdf_filename, model)

        if load_geometry:
            loader = LocalPackageMeshLoader(compas_fab.get(local_package_mesh_folder), "")
            model.load_geometry(loader)

        robot = RobotClass(model, semantics=semantics)

        if client:
            robot.client = client

        return robot

    @classmethod
    def rfl(cls, client=None, load_geometry=True):
        """Returns the RFL robot with 4 ABB irb 4600 and twin-gantry setup.

        This method serves mainly as help for writing examples, so that the code can
        stay short.

        Parameters
        ----------
        client: object
            Backend client. Default is `None`.
        load_geometry: bool, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the robot.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            Newly created instance of the robot.
        """

        robot = cls._load_library_model(
            urdf_filename=compas_fab.get("robot_library/rfl/urdf/robot_description.urdf"),
            srdf_filename=compas_fab.get("robot_library/rfl/robot_description_semantic.srdf"),
            local_package_mesh_folder="robot_library/rfl",
            client=client,
            load_geometry=load_geometry,
        )

        return robot

    @classmethod
    def ur5(cls, client=None, load_geometry=True):
        """Returns a UR5 robot.

        Parameters
        ----------
        client: object
            Backend client. Default is `None`.
        load_geometry: bool, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the robot.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            Newly created instance of the robot.
        """

        robot = cls._load_library_model(
            urdf_filename=compas_fab.get("robot_library/ur5_robot/urdf/robot_description.urdf"),
            srdf_filename=compas_fab.get("robot_library/ur5_robot/robot_description_semantic.srdf"),
            local_package_mesh_folder="robot_library/ur5_robot",
            client=client,
            load_geometry=load_geometry,
        )

        return robot

    @classmethod
    def abb_irb4600_40_255(cls, client=None, load_geometry=True):
        """Returns a ABB irb4600-40/2.55 robot.

        Parameters
        ----------
        client: object
            Backend client. Default is `None`.
        load_geometry: bool, optional
            Default is `True`, which means that the geometry is loaded.
            `False` can be used to speed up the creation of the robot.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            Newly created instance of the robot.
        """

        robot = cls._load_library_model(
            urdf_filename=compas_fab.get("robot_library/abb_irb4600_40_255/urdf/robot_description.urdf"),
            srdf_filename=compas_fab.get("robot_library/abb_irb4600_40_255/robot_description_semantic.srdf"),
            local_package_mesh_folder="robot_library/abb_irb4600_40_255",
            client=client,
            load_geometry=load_geometry,
        )

        return robot


if __name__ == "__main__":
    robot = RobotLibrary.rfl(load_geometry=True)
    robot.info()
    robot = RobotLibrary.ur5(load_geometry=True)
    robot.info()
