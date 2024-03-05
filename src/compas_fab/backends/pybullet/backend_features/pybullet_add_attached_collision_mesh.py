from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import itertools
import os

from compas.geometry import Frame
from compas_robots.model import Inertia
from compas_robots.model import Inertial
from compas_robots.model import Joint
from compas_robots.model import Mass

from compas_fab.backends.interfaces import AddAttachedCollisionMesh
from compas_fab.utilities import LazyLoader

pybullet = LazyLoader("pybullet", globals(), "pybullet")


__all__ = [
    "PyBulletAddAttachedCollisionMesh",
]


class PyBulletAddAttachedCollisionMesh(AddAttachedCollisionMesh):
    """Callable to add a collision mesh and attach it to the robot."""

    def __init__(self, client):
        self.client = client

    def add_attached_collision_mesh(self, attached_collision_mesh, options=None):
        """Add a collision mesh and attach it to the robot.

        Parameters
        ----------
        attached_collision_mesh : :class:`compas_fab.robots.AttachedCollisionMesh`
            Object containing the collision mesh to be attached.
        options : dict
            Dictionary containing the following key-value pairs:

            - ``"robot"``: (:class:`compas_fab.robots.Robot`) Robot instance
              to which the object should be attached.
            - ``"mass"``: (:obj:`float`) The mass of the attached collision
              object.  Defaults to ``1``.
            - ``"inertia"``: (:obj:`list`) The elements of the inertia matrix
              of the attached collision object given as
              ``[<ixx>, <ixy>, <ixz>, <iyy>, <iyz>, <izz>]``.  Defaults to
              ``[1., 0., 0., 1., 0., 1.]``.
            - ``"inertial_origin"``: (:class:`compas.geometry.Frame`) This is
              the pose of the inertial reference frame, relative to the link
              reference frame. Defaults to
              :class:`compas.geometry.Frame.worldXY()`.
            - ``"collision_origin"``: (:class:`compas.geometry.Frame`) This is
              the pose of the collision reference frame, relative to the link
              reference frame. Defaults to
              :class:`compas.geometry.Frame.worldXY()`.
            - ``"concavity"``: (:obj:`bool`) When ``False`` (the default),
              the mesh will be loaded as its convex hull for collision checking purposes.
              When ``True``, a non-static mesh will be decomposed into convex parts using v-HACD.

        Returns
        -------
        ``None``
        """
        robot = options["robot"]
        self.client.ensure_cached_robot_geometry(robot)

        mass = options.get("mass", 1.0)
        concavity = options.get("concavity", False)
        inertia = options.get("inertia", [1.0, 0.0, 0.0, 1.0, 0.0, 1.0])
        inertial_origin = options.get("inertial_origin", Frame.worldXY())
        collision_origin = options.get("collision_origin", Frame.worldXY())

        cached_robot_model = self.client.get_cached_robot(robot)

        # add link
        mesh = attached_collision_mesh.collision_mesh.mesh
        name = attached_collision_mesh.collision_mesh.id
        mesh_file_name = name + ".obj"
        mesh_fp = os.path.join(self.client._cache_dir.name, mesh_file_name)
        mesh.to_obj(mesh_fp)
        mesh_fp = self.client._handle_concavity(mesh_fp, self.client._cache_dir.name, concavity, mass, name)
        link = cached_robot_model.add_link(name, visual_meshes=[mesh], collision_meshes=[mesh])
        mass_urdf = Mass(mass)
        inertia_urdf = Inertia(*inertia)
        inertial_origin_urdf = Frame(inertial_origin.point, inertial_origin.xaxis, inertial_origin.yaxis)
        inertial_urdf = Inertial(inertial_origin_urdf, mass_urdf, inertia_urdf)
        link.inertial = inertial_urdf
        collision_origin_urdf = Frame(collision_origin.point, collision_origin.xaxis, collision_origin.yaxis)
        for element in itertools.chain(link.visual, link.collision):
            element.geometry.shape.filename = mesh_fp
            element.origin = collision_origin_urdf

        # add joint
        parent_link = cached_robot_model.get_link_by_name(attached_collision_mesh.link_name)
        cached_robot_model.add_joint(name + "_fixed_joint", Joint.FIXED, parent_link, link)

        self.client.reload_from_cache(robot)
