from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.geometry import centroid_points
from compas.geometry import cross_vectors
from compas.geometry import normalize_vector
from compas.geometry import centroid_polyhedron
from compas.geometry import volume_polyhedron

# from compas.geometry import bestfit_plane
# from compas.geometry import project_points_plane

from compas.datastructures import Mesh


__all__ = ['Block']


class Block(Mesh):
    """A data structure for the individual blocks of a discrete element assembly.

    Examples
    --------
    .. code-block:: python

        box = Box.from_width_height_depth(2.0, 0.5, 1.0)
        block = Block.from_vertices_and_faces(box.vertices, box.faces)

    """

    __module__ = 'compas_assembly.datastructures'

    def __init__(self):
        super(Block, self).__init__()
        self.attributes.update({'name': 'Block'})

    @classmethod
    def from_polysurface(cls, guid):
        """Class method for constructing a block from a Rhino poly-surface.

        Parameters
        ----------
        guid : str
            The GUID of the poly-surface.

        Returns
        -------
        Block
            The block corresponding to the poly-surface.

        Notes
        -----
        In Rhino, poly-surfaces are organised such that the cycle directions of
        the individual sub-surfaces produce normal vectors that point out of the
        enclosed volume. The normal vectors of the faces of the mesh, therefore
        also point "out" of the enclosed volume.

        """
        from compas_rhino.helpers import mesh_from_surface
        return mesh_from_surface(cls, guid)

    @classmethod
    def from_rhinomesh(cls, guid):
        """Class method for constructing a block from a Rhino mesh.

        Parameters
        ----------
        guid : str
            The GUID of the mesh.

        Returns
        -------
        Block
            The block corresponding to the Rhino mesh.

        """
        from compas_rhino.helpers import mesh_from_guid
        return mesh_from_guid(cls, guid)

    def centroid(self):
        """Compute the centroid of the block.

        Returns
        -------
        point
            The XYZ location of the centroid.

        """
        return centroid_points(
            [self.vertex_coordinates(key) for key in self.vertices()])

    def frames(self):
        """Compute the local frame of each face of the block.

        Returns
        -------
        dict
            A dictionary mapping face identifiers to face frames.

        """
        return {fkey: self.frame(fkey) for fkey in self.faces()}

    def frame(self, fkey):
        """Compute the frame of a specific face.

        Parameters
        ----------
        fkey : hashable
            The identifier of the frame.

        Returns
        -------
        frame
            The frame of the specified face.

        """
        xyz = self.face_coordinates(fkey)
        o = xyz[0]
        # o = self.face_centroid(fkey)
        w = self.face_normal(fkey)
        u = [xyz[1][i] - o[i] for i in range(3)]
        v = cross_vectors(w, u)
        uvw = normalize_vector(u), normalize_vector(v), normalize_vector(w)
        return o, uvw

    # def frame_offset(self, fkey):
    #     """Compute the frame with offset"""

    #     # TODO need to be re-writen properly.
    #     xyz = self.face_coordinates(fkey)

    #     centroid = centroid_points(xyz)

    #     new_xyz = []
    #     for pt in xyz:
    #         vec = [pt[i] * 0.9 + centroid[i] * 0.1 for i in range(3)]
    #         new_xyz.append(vec)

    #     o = new_xyz[0]
    #     w = self.face_normal(fkey)
    #     u = [new_xyz[1][i] - o[i] for i in range(3)]
    #     v = cross_vectors(w, u)
    #     uvw = normalize_vector(u), normalize_vector(v), normalize_vector(w)
    #     return o, uvw

    # def frames_offset(self):
    #     # TODO need to be clean up
    #     return {fkey: self.frame_offset(fkey) for fkey in self.faces()}

    # def frame_planar(self, fkey):
    #     """Planarize and compute the frame of a specific face.

    #     Parameters
    #     ----------
    #     fkey : hashable
    #         The identifier of the frame.

    #     Returns
    #     -------
    #     frame
    #         The frame of the specified face.

    #     """

    #     xyz = self.face_coordinates(fkey)

    #     b_plane = bestfit_plane(xyz)
    #     b_xyz = project_points_plane(xyz, b_plane)

    #     o = b_xyz[0]
    #     w = b_plane[1]
    #     u = (
    #         b_xyz[2][0] - b_xyz[1][0],
    #         b_xyz[2][1] - b_xyz[1][1],
    #         b_xyz[2][2] - b_xyz[1][2],
    #     )

    #     v = cross_vectors(w, u)
    #     uvw = normalize_vector(u), normalize_vector(v), normalize_vector(w)
    #     return o, uvw, b_xyz, b_plane

    def top(self):
        """Identify the *top* face of the block.

        Returns
        -------
        int
            The identifier of the face.

        Notes
        -----
        The face with the highest centroid is considered the *top* face.

        """
        frames = self.frames()  # key: (o, uvw)
        fkey_centroid = {fkey: self.face_center(fkey) for fkey in self.faces()}
        fkey, _ = sorted(fkey_centroid.items(), key=lambda x: x[1][2])[-1]
        return fkey

    def center(self):
        """Compute the center of mass of the block.

        Returns
        -------
        point
            The center of mass of the block.

        """
        vertices = [self.vertex_coordinates(key) for key in self.vertices()]
        faces = [self.face_vertices(fkey) for fkey in self.faces()]
        return centroid_polyhedron((vertices, faces))

    def volume(self):
        """Compute the volume of the block.

        Returns
        -------
        float
            The volume of the block.

        """
        vertices = [self.vertex_coordinates(key) for key in self.vertices()]
        faces = [self.face_vertices(fkey) for fkey in self.faces()]
        v = volume_polyhedron((vertices, faces))
        return v


# ==============================================================================
# Debugging
# ==============================================================================

if __name__ == "__main__":
    pass
