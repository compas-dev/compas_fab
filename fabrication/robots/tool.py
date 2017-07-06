'''
Created on 01.03.2017

@author: rustr
'''

from compas_fabrication.fabrication.geometry import Frame, Transformation

class Tool(object):
    """
    This is the base class for tools / robot's end-effectors.
    It consists of:
    - geometry (meshes)
    - frame
    - transformation matrix
    """
    
    def __init__(self, tcp_frame):
                
        self.meshes = []
        self.tool0_frame = Frame.worldXY()
        self.tcp_frame = tcp_frame
        self.transformation_tcp_tool0 = Transformation.from_frame_to_frame(self.tcp_frame, self.tool0_frame)
        self.transformation_tool0_tcp = Transformation.from_frame_to_frame(self.tool0_frame, self.tcp_frame)
    
    def set_meshes(self, meshes):
        
        self.meshes = meshes
        