from typing import TYPE_CHECKING
from typing import Optional

from compas_fab.backends.interfaces import SetRobotCell
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState

# Optional high-performance acceleration plugin
try:
    import compas_forge
    COMPAS_FORGE_AVAILABLE = True
except ImportError:
    COMPAS_FORGE_AVAILABLE = False

if TYPE_CHECKING:
    from compas_fab.backends import PyBulletClient


def _register_forge_mesh(mesh_id: str, geometry) -> None:
    """Helper to extract flat buffers and register mesh to the compas-forge Rust core."""
    if not COMPAS_FORGE_AVAILABLE or geometry is None:
        return
    
    from compas.datastructures import Mesh
    
    # If geometry is already a COMPAS Mesh
    if isinstance(geometry, Mesh):
        compas_forge.register_mesh_to_cache(mesh_id, geometry)
    
    # If geometry is a basic COMPAS shape (like Box, Sphere, Cylinder, etc.), convert to Mesh
    elif hasattr(geometry, 'to_vertices_and_faces'):
        vertices, faces = geometry.to_vertices_and_faces()
        temp_mesh = Mesh.from_vertices_and_faces(vertices, faces)
        compas_forge.register_mesh_to_cache(mesh_id, temp_mesh)


class PyBulletSetRobotCell(SetRobotCell):
    def set_robot_cell(self, robot_cell: RobotCell, robot_cell_state: RobotCellState = None, options: Optional[dict] = None):
        """Pass the models in the robot cell to the Pybullet client.

        The client keeps the robot cell models in memory and uses them for planning.
        Calling this method will override the previous robot cell in the client.
        It should be called only if the robot cell models have changed.

        The RobotCell object passed to this method should not be modified after
        calling this method, as the client keeps a reference to it.

        Parameters
        ----------
        robot_cell
            The robot cell object containing the robot, tools, and other objects.
        robot_cell_state
            The state of the robot cell, including the robot's configuration.
        options
            Dictionary containing additional options.
            Unused at the moment.

        """

        if not isinstance(robot_cell, RobotCell):
            raise TypeError("robot_cell should be an instance of RobotCell instead of: {}".format(type(robot_cell).__name__))

        client: PyBulletClient = self.client

        # TODO: Check for new, modified and removed objects compared to the
        # TODO: previous robot cell state and update the PyBullet world accordingly
        # previous_robot_cell = client.robot_cell or RobotCell()

        # At the moment, all previous object in the PyBullet world are removed
        # and the new robot cell is added to the PyBullet world

        # Remove all objects from the PyBullet world

        for tool_id in list(client.tools_puids.keys()):
            client._remove_tool(tool_id)
        # client.tools_puids = {}
        for rigid_body_id in list(client.rigid_bodies_puids.keys()):
            client._remove_rigid_body(rigid_body_id)
        # client.rigid_bodies_puids = {}

        # Clear any existing Rust cache registry on loading a new cell
        if COMPAS_FORGE_AVAILABLE:
            compas_forge.clear_mesh_cache()

        # Add the robot cell to the PyBullet world
        for name, tool_model in robot_cell.tool_models.items():
            client._add_tool(name, tool_model)
            
            # Automatically register tool link collision meshes to the Rust cache core
            if COMPAS_FORGE_AVAILABLE:
                for link in tool_model.links:
                    for i, collision in enumerate(link.collision):
                        mesh_id = "tool_{}_{}_{}".format(name, link.name, i)
                        _register_forge_mesh(mesh_id, collision.geometry)

        for name, rigid_body in robot_cell.rigid_body_models.items():
            client._add_rigid_body(name, rigid_body)
            
            # Automatically register stationary rigid body obstacles to the Rust cache core
            if COMPAS_FORGE_AVAILABLE:
                mesh_id = "body_{}".format(name)
                geom = getattr(rigid_body, 'geometry', None) or getattr(rigid_body, 'mesh', None)
                if geom:
                    _register_forge_mesh(mesh_id, geom)

        # Feed the robot to the client
        if robot_cell.robot_model:
            robot_cell.ensure_geometry()
            robot_cell.ensure_semantics()
            client._set_robot(
                robot_cell.robot_model,
                robot_cell.robot_semantics,
            )
            
            # Automatically register robot link collision meshes to the Rust cache core
            if COMPAS_FORGE_AVAILABLE:
                for link in robot_cell.robot_model.links:
                    for i, collision in enumerate(link.collision):
                        mesh_id = "robot_{}_{}".format(link.name, i)
                        _register_forge_mesh(mesh_id, collision.geometry)

        # Keep a copy of the robot cell in the client
        # from copy import deepcopy
        # client._robot_cell = deepcopy(robot_cell)

        # NOTE: The current implementation of `compas_robots.RobotModel`` is very slow to
        #       copy or deepcopy. Making the robot cell also slow to copy or deepcopy.
        #       It is better to keep a reference to the robot cell.
        client._robot_cell = robot_cell

        # If a robot cell state is provided, update the client's robot cell state
        if robot_cell_state:
            self.set_robot_cell_state(robot_cell_state)