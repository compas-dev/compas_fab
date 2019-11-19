from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.geometry import Frame
from compas.datastructures import Mesh


class Tool(object):
    """Represents a tool to be attached to the robot's flange.

    Attributes
    ----------
    visual : :class:`compas.datastructures.Mesh`
        The visual mesh of the tool. 
    frame : :class:`compas.geometry.Frame`
        The frame of the tool in tool0 frame.
    name : str
        The name of the `Tool`. Defaults to 'attached_tool'.
    collision : :class:`compas.datastructures.Mesh`
        The collision mesh representation of the tool.
    
    Examples
    --------
    >>>


    """
    
    def __init__(self, visual, frame_in_tool0_frame, name="attached_tool", 
                 collision=None):
        self.visual = visual
        self.frame = frame_in_tool0_frame
        self.name = name
        self.collision = collision
    
    @property
    def collision(self):
        """The collision mesh representation of the tool."""
        return self._collision

    @collision.setter
    def collision(self, collision):
        self._collision = collision or self.visual
    
    def attached_collision_mesh(self, robot):
        pass

if __name__ == "__main__":
    import compas_fab
    mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
    frame = Frame([0.14, 0, 0], [0, 1, 0], [0, 0, 1])
    tool = Tool(mesh, frame)
    tool.collision = mesh
    """
    acm = robot.set_end_effector(mesh, frame)
    frame_tcf = Frame((-0.309, -0.046, -0.266), (0.276, 0.926, -0.256), (0.879, -0.136, 0.456))
    robot.to_tool0_frame(frame_tcf)
    Frame(Point(-0.363, 0.003, -0.147), Vector(0.388, -0.351, -0.852), Vector(0.276, 0.926, -0.256))
    tool = Tool()
    """