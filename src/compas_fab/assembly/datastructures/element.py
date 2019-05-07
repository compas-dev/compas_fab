from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.geometry import Frame
from compas.datastructures import Mesh

__all__ = ['Element']

class Element(object):
    """A data structure for the individual elements of a discrete element assembly.

    Attributes
    ----------
    name : str
        The name that identifies the element.

    frame : :class:`compas.geometry.Frame`
        The frame of the element.

    grip_frame : :class:`compas.geometry.Frame`
        The gripping frame of the element.
        Default is the element frame.

    data : dict
        The data representing the element.
        The dict has the following structure:
        * 'name'       => str
        * 'frame'      => :class:`compas.geometry.Frame`
        * 'grip_frame' => :class:`compas.geometry.Frame`

    Examples
    --------
    from compas_fab.assembly import Element
    from compas.datastructures import Mesh

    frame = Frame.worldXY()
    element = Element(frame)

    mesh = Mesh.from_polyhedron(4)
    element = Element.from_mesh(mesh)

    ...

    """

    __module__ = 'compas_fab.assembly.datastructures'


    def __init__(self, frame):
        super(Element, self).__init__()
        self._frame = None
        self._grip_frame = None

        self.name = None
        self.frame = frame
        self.grip_frame = frame

        self.mesh = None
        self.centroid = frame[0]

    @property
    def frame(self):
        """Frame: The element's frame."""
        return self._frame

    @frame.setter
    def frame(self, frame):
        self._frame = Frame(frame[0], frame[1], frame[2])

    @property
    def grip_frame(self):
        """grip_frame: The element's gripping frame."""
        return self._grip_frame

    @grip_frame.setter
    def grip_frame(self, frame):
        self._grip_frame = Frame(frame[0], frame[1], frame[2])

    # --------------------------------------------------------------------------
    # customisation
    # --------------------------------------------------------------------------

    def __str__(self):
        """Generate a readable representation of the data of the element."""
        return str(self.data)

    # --------------------------------------------------------------------------
    # factory
    # --------------------------------------------------------------------------

    @classmethod
    def from_data(cls, data):
        """Construct an element from its data representation.

        Parameters
        ----------
        data : :obj:`dict`
            The data dictionary.

        Returns
        -------
        Element
            The constructed element.

        Examples
        --------
        from compas.geometry import Frame
        from compas_fab.assembly.datastructures import Element

        data = {frame': Frame.worldXY().data, 'grip_frame': Frame.worldXY().data}
        element = Element.from_data(data)

        """
        element = cls(Frame.worldXY())
        element.data = data
        return element

    @property
    def data(self):
        """Returns the data dictionary that represents the element.

        Returns
        -------
        dict
            The element data.

        Examples
        --------
        from compas.geomtry import Frame
        from compas_fab.assembly import Element

        element = Element(Frame.worldXY())

        print(element.data)
        ...

        """
        return {'name'      : self.name,
                'frame'     : self.frame.data,
                'grip_frame': self.grip_frame.data,
                'mesh'      : self.mesh,
                'centroid'  : self.centroid
                }

    @data.setter
    def data(self, data):
        self.name = data['name']
        self.frame = Frame.from_data(data['frame'])
        self.grip_frame = data['grip_frame']
        self.centroid = data['centroid']

    def to_data(self):
        """Returns the data dictionary that represents the element.

        Returns
        -------
        dict
            The element data.

        Examples
        --------
        from compas.geomtry import Frame
        from compas_fab.assembly import Element

        element = Element(Frame.worldXY())

        print(element.to_data)

        """
        return self.data

    @classmethod
    def from_mesh(cls, mesh=None, guid=None): # classmethod or instance method?
        """Class method for constructing an element from a COMPAS mesh # or a Rhino mesh

        Parameters
        ----------

        mesh : :class:`compas.geometry.Mesh`
            The COMPAs mesh.

        # guid : str
        #     The GUID of the rhino mesh.

        Returns
        -------
        Element
            The element corresponding to the input mesh.

        """
        # from compas_rhino.helpers import mesh_from_guid

        # if guid is not None:
            # mesh = mesh_from_guid(guid)
            # mesh = Mesh.from_obj(compas.get('faces.obj'))

        centroid = mesh.centroid()
        frame_centroid = Frame(centroid, [1, 0, 0], [0, 1, 0])
        element = Element(frame_centroid)
        element.mesh = mesh

        return element

    # --------------------------------------------------------------------------
    # attributes
    # --------------------------------------------------------------------------

    def centroid(self, frame):
        """Compute the centroid of the element.

        Returns
        -------
        point
            The XYZ location of the centroid.

        """
        self.centroid = frame[0]

        return self.centroid

    # --------------------------------------------------------------------------
    # representation
    # --------------------------------------------------------------------------

    def __repr__(self):
        return 'Element({}, {})'.format(self.frame, self.grip_frame)

    # --------------------------------------------------------------------------
    # helpers
    # --------------------------------------------------------------------------

    def copy(self):
        """Make a copy of this ``Element``.

        Returns
        -------
        Element
            The copy.
        """
        cls = type(self)
        return cls(self.frame.copy, self.grip_frame.copy)

# ==============================================================================
# Main
# ==============================================================================

if __name__ == "__main__":

    from compas.datastructures import Mesh

    frame = Frame.worldXY()
    element = Element(frame)

    mesh = Mesh.from_polyhedron(4)
    element = Element.from_mesh(mesh)

    print(element.data)
    element.name = "Nooooone"
    print(element.data)
