from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import time

import compas
import compas_rhino

from compas_fab.artists import BaseRobotArtist
from compas.utilities import pairwise
from compas.geometry import centroid_polygon

try:
    import Rhino.Geometry
    import scriptcontext as sc
    from System.Drawing import Color
    from Rhino.DocObjects.ObjectColorSource import ColorFromObject
    from Rhino.DocObjects.ObjectColorSource import ColorFromLayer
    from Rhino.DocObjects.ObjectMaterialSource import MaterialFromObject
except ImportError:
    compas.raise_if_ironpython()

__all__ = [
    'RobotArtist',
]


class RobotArtist(BaseRobotArtist):
    """Visualizer for robots inside a Rhino environment.

    Parameters
    ----------
    robot : compas.robots.RobotModel
        Robot model.
    layer : str, optional
        The name of the layer that will contain the robot meshes.
    """

    def __init__(self, robot, layer=None):
        super(RobotArtist, self).__init__(robot)
        self.layer = layer

    def transform(self, native_mesh, transformation):
        T = xform_from_transformation(transformation)
        native_mesh.Transform(T)

    def draw_geometry(self, geometry, name=None, color=None):
        # Imported colors take priority over a the parameter color
        if 'mesh_color.diffuse' in geometry.attributes:
            color = geometry.attributes['mesh_color.diffuse']

        key_index = geometry.key_index()
        vertices = geometry.get_vertices_attributes('xyz')
        faces = [[key_index[key] for key in geometry.face_vertices(fkey)] for fkey in geometry.faces()]
        new_faces = []
        for face in faces:
            f = len(face)
            if f == 3:
                new_faces.append(face + face[-1:])
            elif f == 4:
                new_faces.append(face)
            elif f > 4:
                centroid = len(vertices)
                vertices.append(centroid_polygon([vertices[index] for index in face]))
                for a, b in pairwise(face + face[0:1]):
                    new_faces.append([centroid, a, b, b])
            else:
                continue

        mesh = Rhino.Geometry.Mesh()

        if name:
            mesh.UserDictionary.Set('MeshName', name)

        if color:
            r, g, b, a = color
            mesh.UserDictionary.Set('MeshColor.R', r)
            mesh.UserDictionary.Set('MeshColor.G', g)
            mesh.UserDictionary.Set('MeshColor.B', b)
            mesh.UserDictionary.Set('MeshColor.A', a)

        for v in vertices:
            mesh.Vertices.Add(*v)
        for face in faces:
            mesh.Faces.AddFace(*face)

        mesh.Normals.ComputeNormals()
        mesh.Compact()

        return mesh

    def _enter_layer(self):
        self._previous_layer = None

        if self.layer:
            if not compas_rhino.rs.IsLayer(self.layer):
                compas_rhino.create_layers_from_path(self.layer)
            self._previous_layer = compas_rhino.rs.CurrentLayer(self.layer)

        compas_rhino.rs.EnableRedraw(False)

    def _exit_layer(self):
        if self.layer and self._previous_layer:
            compas_rhino.rs.CurrentLayer(self._previous_layer)

        self.redraw()

    def draw_collision(self):
        collisions = super(RobotArtist, self).draw_collision()
        collisions = list(collisions)

        self._enter_layer()

        for mesh in collisions:
            self._add_mesh_to_doc(mesh)

        self._exit_layer()

    def draw_visual(self):
        visuals = super(RobotArtist, self).draw_visual()
        visuals = list(visuals)

        self._enter_layer()

        for mesh in visuals:
            self._add_mesh_to_doc(mesh)

        self._exit_layer()

    def redraw(self, timeout=None):
        """Redraw the Rhino view.

        Parameters
        ----------
        timeout : float, optional
            The amount of time the artist waits before updating the Rhino view.
            The time should be specified in seconds.
            Default is ``None``.

        """
        if timeout:
            time.sleep(timeout)

        compas_rhino.rs.EnableRedraw(True)
        compas_rhino.rs.Redraw()

    def clear_layer(self):
        """Clear the main layer of the artist."""
        if self.layer:
            compas_rhino.clear_layer(self.layer)
        else:
            compas_rhino.clear_current_layer()

    def _add_mesh_to_doc(self, mesh):
        guid = sc.doc.Objects.AddMesh(mesh)

        color = None
        if 'MeshColor.R' in mesh.UserDictionary:
            color = [mesh.UserDictionary['MeshColor.R'],
                     mesh.UserDictionary['MeshColor.G'],
                     mesh.UserDictionary['MeshColor.B'],
                     mesh.UserDictionary['MeshColor.A']]
        name = mesh.UserDictionary['MeshName'] if 'MeshName' in mesh.UserDictionary else None

        obj = sc.doc.Objects.Find(guid)

        if obj:
            attr = obj.Attributes
            if color:
                r, g, b, a = [i * 255 for i in color]
                attr.ObjectColor = Color.FromArgb(a, r, g, b)
                attr.ColorSource = ColorFromObject

                material_name = 'robotartist.{:.2f}_{:.2f}_{:.2f}_{:.2f}'.format(r, g, b, a)
                material_index = sc.doc.Materials.Find(material_name, True)

                # Material does not exist, create it
                if material_index == -1:
                    material_index = sc.doc.Materials.Add()
                    material = sc.doc.Materials[material_index]
                    material.Name = material_name
                    material.DiffuseColor = attr.ObjectColor
                    material.CommitChanges()

                attr.MaterialIndex = material_index
                attr.MaterialSource = MaterialFromObject
            else:
                attr.ColorSource = ColorFromLayer

            if name:
                attr.Name = name

            obj.CommitChanges()


# TODO: Move to compas_rhino.geometry
def xform_from_transformation(transformation):
    """Creates a Rhino Transform instance from a :class:`Transformation`.

    Args:
        transformation (:class:`Transformation`): the transformation.

    Returns:
        (:class:`Rhino.Geometry.Transform`)
    """
    transform = Rhino.Geometry.Transform(1.0)
    for i in range(0, 4):
        for j in range(0, 4):
            transform[i, j] = transformation[i, j]
    return transform
