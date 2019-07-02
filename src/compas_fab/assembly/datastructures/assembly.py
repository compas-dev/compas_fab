from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.datastructures.network import Network
from compas_fab.assembly.datastructures.element import Element

__all__ = ['Assembly']


class Assembly(Network):
    """A data structure for discrete element assemblies.

    An assembly is a network of assembly elements.
    Each vertex of the network represents an element of the assembly.
    Each edge of the network represents an interface between two assembly elements.

    # or inverted?
    # Each edge of the network represents an element of the assembly.
    # Each vertex of the network represents an interface between two assembly elements.

    # @tvm: should an assembly be composed of a network attribute
    # and a block collection
    # rather than inherit from network
    # and add inconsistent stuff to that interface?

    Parameters
    ----------
    elements : list of :class:`compas_fab.assembly.datastructures.Element`, optional
        A list of assembly elements.

    attributes : dict, optional
        User-defined attributes of the assembly.
        Built-in attributes are:

        * name (str) : 'Assembly'

    default_vertex_attributes : dict, optional
        User-defined default attributes of the vertices of the network.
        Since the vertices of the assembly network represent the individual
        elements, the built-in attributes are:

        * is_placed (bool)  : False
        * is_support (bool) : False # confusing name: all elements are supports of each other > revise
        * course(int)       : None # not generic enough

    default_edge_attributes : dict, optional
        User-defined default attributes of the edges of the network.
        Since the edges of the assembly network represent the interfaces between
        the individual elements, the built-in attributes are:

        * interface_points (list) : None
        * interface_type ({'face_face', 'face_edge', 'face_vertex'}) : None
        * interface_size (float) : None
        * interface_uvw (list) : None
        * interface_origin (list) : None
        * interface_forces (list) : None

    Examples
    --------
    .. code-block:: python

        from compas.datastructures import Mesh
        from compas_fab.assembly import Assembly
        from compas_fab.assembly import Element

        assembly = Assembly()
        mesh = Mesh.from_polyhedron(4)

        for i in range(2):
            element = Element.from_mesh(mesh)
            assembly.add_element(element)

        print(assembly.summary())

    """

    __module__ = 'compas_assembly.datastructures'

    def __init__(self, elements=None, attributes=None, default_vertex_attributes=None,
    default_edge_attributes=None):
        super(Assembly, self).__init__()

        # self.elements = {}
        # self.attributes.update({'name': 'Assembly'})
        # if attributes is not None:
        #     self.attributes.update(attributes)
        #
        # self.default_vertex_attributes.update({
        #     'is_placed': False
        #     # 'is_support': False, # confusing name: all elements are supports of each other?
        #     # 'course': None, # not generic enough, category for location?
        # })
        # if default_vertex_attributes is not None:
        #     self.default_vertex_attributes.update(default_vertex_attributes)
        #
        # self.default_edge_attributes.update({
        #     'interface_points': None,
        #     'interface_type': None,
        #     'interface_size': None,
        #     'interface_uvw': None,
        #     'interface_origin': None,
        #     'interface_forces': None,
        # })
        # if default_edge_attributes is not None:
        #     self.default_edge_attributes.update(default_edge_attributes)
        #
        # if elements:
        #     for element in elements:
        #         self.add_element(element)

    # --------------
    # constructors
    # --------------
    @classmethod
    def from_network(cls, net):
        """generate an assembly network from a network, which only contains
        abstract connectivity info.

        The generated assembly assumes pairwise virtual joint model. For example,
        if three elements are joined together, calling `from_network` will create
        three virtual joints on J(e2, e2), J(e2, e3), J(e1, e3), instead of
        J(e1, e2, e3).

        # TODO: make a `merge` fn for the virtual joint object.

        Parameters
        ----------
        cls : type
        net : compas Network
            a network that contains connectivity info between unit elements
            The network's vertices represent unit assembly elements, edges
            represents connectivity between parts.

        Returns
        -------
        Assembly
            an assembly object that contains only abstract connecvity info.

        """
        as_net = cls()
        as_net.vertex = net.vertex
        as_net.edge = net.edge
        as_net.halfedge = net.halfedge
        as_net.attributes = net.attributes
        # TODO: create default virtual joints between objects
        return as_net

    @classmethod
    def from_network_and_assembly_objects(cls, net, objs):
        assert(net.number_of_vertices()==len(objs), \
            "assembly objects and network vertices not aligned!")
        as_net = cls.from_network(net)

        # assign assembly object attribute
        for i, vert_key in enumerate(as_net.vertices()):
            objs[i].id = i
            as_net.set_vertex_attribute(vert_key, "assembly_object", objs[i])
        return as_net

    def add_element(self, element, key=None, attr_dict=None, **kwattr):
        """Add an element to the assembly.

        Parameters
        ----------
        element : compas_fab.assembly.datastructures.element
            The element to add.
        attr_dict : dict, optional
            A dictionary of element attributes.
            Default is ``None``.

        Returns
        -------
        hashable
            The identifier of the element.

        Notes
        -----
        The element is added as a vertex in the assembly data structure. > decide if it should be edge
        The XYZ coordinates of the vertex are the coordinates of the centroid of the element. > decide if it should be edge

        """
        pass

if __name__ == "__main__":
    pass
    # from compas.datastructures import Mesh
    # from compas_fab.assembly import Element

    # assembly = Assembly()
    # mesh = Mesh.from_polyhedron(4)

    # for i in range(2):
    #     element = Element.from_mesh(mesh)
    #     assembly.add_element(element)

    # print(assembly.summary())
