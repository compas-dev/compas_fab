from compas.data import Data
from compas.datastructures import Mesh  # noqa: F401

try:
    from typing import List  # noqa: F401
    from typing import Optional  # noqa: F401
except ImportError:
    pass


class WorkpieceModel(Data):
    """Represents the immutable properties of a workpiece. Used by the FabricationProcess.

    The state (mutable properties) of the workpiece (e.g. frame, is_hidden) is defined using
    the :class:`compas_fab.planning.WorkpieceState` class.

    Attributes
    ----------
    name : str
        The name of the workpiece.

    collision_meshes : :class:`compas.datastructures.Mesh`
        A Mesh for modeling the collision geometry of the workpiece.
        It is used for collision checking during planning.
        The mesh should be modeled in the workpiece's local coordinate frame.
        The location of the workpiece in the world coordinate frame is defined by the WorkpieceState.
        The mesh must be a closed mesh (i.e. watertight) without self intersections.
        Note that many planning backends will triangulate the mesh before sending
        it to the backend. To avoid descrepancies between the defined meshes and the triangulated
        mesh in the backends, it is recommended to triangulate the mesh before creating the WorkpieceModel.

    visual_meshes : list[:class:`compas.datastructures.Mesh`], optional
        A list of Meshes for modeling the visual appearance of the workpiece.
        It is used for visualization in the viewer.
        If not specified, the collision_meshes is used for visualization.
    """

    def __init__(self, name, collision_meshes, visual_meshes=None):
        super(WorkpieceModel, self).__init__()
        self.name = name  # type: str
        self.collision_meshes = collision_meshes  # type: List[Mesh]
        self.visual_meshes = visual_meshes  # type: Optional[List[Mesh]]

    def __str__(self):
        return "Workpiece(name='{}')".format(self.name)

    def __repr__(self):
        return self.__str__()

    def to_data(self):
        """Returns the data dictionary that represents the workpiece.

        Returns
        -------
        dict
            The workpiece's data.
        """
        return {
            "name": self.name,
            "collision_meshes": self.collision_meshes,
            "visual_meshes": self.visual_meshes,
        }

    @classmethod
    def from_data(cls, data):
        """Constructs a workpiece from its data representation.

        Parameters
        ----------
        data : dict
            The data dictionary.

        Returns
        -------
        :class:`compas_fab.robots.WorkpieceModel`
            The constructed workpiece.
        """
        name = data["name"]
        collision_meshes = data["collision_meshes"]
        visual_meshes = data["visual_meshes"]
        return cls(name, collision_meshes, visual_meshes)
