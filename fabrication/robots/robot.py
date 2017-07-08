'''
Created on 01.03.2017

@author: rustr
'''

from compas_fabrication.fabrication.geometry import Frame, Transformation


class Robot(object):
    """
    This is the base class for all robots.
    It consists of:
    - a geometry (meshes)
    - a base: describes where the robot is attached to. This can be also a movable base: e.g. linear axis 
    - a basis frame, the frame it resides, e.g. Frame.worldXY()
    - a transformation matrix to get coordinated represented in RCS
    - a transformation matrix to get coordinated represented in WCS
    - a tool, the end-effector
    - communication: e.g. delegated by a client instance
    - workspace: brep ?
 
    self.configuration = [0,0,0,0,0,0]
    self.tcp_frame = tcp_frame
    self.tool0_frame = tool0_frame
    
    # transform world to robot origin
    self.T_W_R = rg.Transform.PlaneToPlane(Frame.worldXY, self.basis_frame)
    # transform robot to world
    self.T_R_W = rg.Transform.PlaneToPlane(self.basis_frame, Frame.worldXY)
    """
    
    def __init__(self):
        
        self.model = [] # a list of meshes
        self.model_loaded = False
        self.basis_frame = None
        self.transformation_RCS_WCS = None
        self.transformation_WCS_RCS = None
        self.set_base(Frame.worldXY())
        self.tool = None
        self.configuration = None
        self.tool0_frame = Frame.worldXY()
        
    def load_model(self):
        self.model_loaded = True
        
    
    def set_base(self, base_frame):
        self.base_frame = base_frame
        # transformation matrix from world coordinate system to robot coordinate system
        self.transformation_RCS_WCS = Transformation.from_frame_to_frame(Frame.worldXY(), self.base_frame)
        # transformation matrix from robot coordinate system to world coordinate system
        self.transformation_WCS_RCS = Transformation.from_frame_to_frame(self.base_frame, Frame.worldXY())
        # modify joint axis !
        
    
    def set_tool(self, tool):
        self.tool = tool
        
    def get_robot_configuration(self):
        pass
        
    @property
    def tcp_frame(self):
        # read from tool
        if not self.tool:
            return self.tool0_frame
    
    def forward_kinematics(self, q):
        """
        Calculate the tcp frame according to the joint angles q.
        """
        raise NotImplementedError
    
    def inverse_kinematics(self, tcp_frame_RCS):
        """
        Calculate solutions (joint angles) according to the queried tcp frame
        (in RCS).
        """
        raise NotImplementedError
    
    def get_frame_in_RCS(self, frame_WCS):
        """
        Transform the frame in world coordinate system (WCS) into a frame in 
        robot coordinate system (RCS), which is set by the robots' basis frame.
        """
        frame_RCS = frame_WCS.transform(self.transformation_WCS_RCS)
        return frame_RCS
    
            
    def get_tool0_frame_from_tcp_frame(self, frame_tcp):
        """
        Get the tool0 frame (frame at robot) from the tool frame (tcp),
        according to the set tool.
        
        """
        frame_tool0 = Frame()
        return frame_tool0
    

        """
        tool_plane_in_RCS = self.get_tool_plane_in_RCS(tp_WCS)

        T_TP_in_zero_W = rg.Transform.PlaneToPlane(self.tool.plane, rg.Plane.WorldXY)
        
        tcp_plane_in_RCS = rg.Plane.WorldXY
        tcp_plane_in_RCS.Transform(T_TP_in_zero_W)
        
        T_W_TP_in_RCS = rg.Transform.PlaneToPlane(rg.Plane.WorldXY, tool_plane_in_RCS)
        tcp_plane_in_RCS.Transform(T_W_TP_in_RCS)
        
        return tcp_plane_in_RCS
        """
    


if __name__ == "__main__":
    
    base_frame = Frame([-636.57, 370.83, 293.21], [0.00000, -0.54972, -0.83535], [0.92022, -0.32695, 0.21516])
    robot = Robot()
    robot.set_base(base_frame)
    T1 = robot.transformation_WCS_RCS
    T2 = robot.transformation_RCS_WCS
    print T1 * T2
    
    
    
    