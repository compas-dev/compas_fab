from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.geometry import Frame
from compas.geometry import Sphere
from compas.geometry import Box

__all__ = ['Constraint', 'JointConstraint', 'OrientationConstraint', 
           'BoundingVolume', 'PositionConstraint']

class Constraint(object):
    """Base class for robot constraints.
    
    Attributes
    ----------
    weight: float, optional
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important. Defaults to 1.
    """
    JOINT = 1
    POSITION = 2
    ORIENTATION = 3
    possible_types = [JOINT, POSITION, ORIENTATION]

    def __init__(self, type, weight=1.):
        if type not in self.possible_types:
            raise ValueError("Type must be %d, %d or %d" % tuple(self.possible_types))
        self.type = type
        self.weight = weight

class JointConstraint(Constraint):
    """Constrains the value of a joint to be within a certain bound.

    Attributes
    ----------
    joint_name: string
        The name of the joint this contraint refers to.
    value: float
        The targeted value for that joint.
    tolerance: float
        The bound to be achieved is [value - tolerance, value + tolerance]
    weight: float, optional
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important. Defaults to 1.
    """
    def __init__(self, joint_name, value, tolerance=0., weight=1.):
        super(JointConstraint, self).__init__(self.JOINT, weight)
        self.joint_name = joint_name
        self.value = value
        self.tolerance = tolerance

class OrientationConstraint(Constraint):
    """Constrains a link to be within a certain orientation.

    Attributes
    ----------
    link_name: string
        The name of the link this contraint refers to.
    euler_angles: list of float
        The desired orientation of the link specified by euler angles, 
        describing rotations about static 'xyz' axes.
    tolerances: list of float, optional
        Error tolerances ti for each of the euler angles ai. The respective 
        bound to be achieved is [ai - ti, ai + ti]. Defaults to [0.,0.,0.].
    weight: float, optional
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important. Defaults to 1.
    """
    def __init__(self, link_name, euler_angles, tolerances=None, weight=1.):
        super(OrientationConstraint, self).__init__(self.ORIENTATION, weight)
        self.link_name = link_name
        self.euler_angles = [float(a) for a in list(euler_angles)]
        self.tolerances = [float(a) for a in list(tolerances)] if tolerances else [0.,0.,0.]

class Box(object):
    """Represents a box.

    Attributes
    ----------
    frame: :class:`compas.geometry.Frame`
        The position and orientation of the box as frame.
    xsize: float
        The size of the box in the frame's x direction.
    ysize: float
        The size of the box in the frame's y direction.
    zsize: float
        The size of the box in the frame's z direction.
    """
    def __init__(self, frame, xsize, ysize, zsize):
        self.frame = frame
        self.xsize = float(xsize)
        self.ysize = float(ysize)
        self.zsize = float(zsize)
        
class Sphere(object):
    """Represents a sphere.

    Attributes
    ----------
    center: :class:`compas.geometry.Point` or `list` of `float`
        The center of the sphere.
    radius: float
        The radius of the sphere.
    """
    def __init__(self, center, radius):
        self.center = Point(*list(center))
        self.radius = float(radius)

class BoundingVolume(object):
    """Describes a bounding volume.
    """
    BOX = 1
    SPHERE = 2
    MESH = 3

    def __init__(self, type, volume):
        if type not in [self.BOX, self.SPHERE, self.MESH]:
            raise ValueError("Type must be %d, %d or %d"
                             % (self.BOX, self.SPHERE, self.MESH))
        self.type = type
        self.volume = volume
    @classmethod
    def from_box(cls, box):
        return cls(cls.BOX, box)
    @classmethod
    def from_sphere(cls, sphere):
        return cls(cls.SPHERE, sphere)
    @classmethod
    def from_mesh(cls, mesh):
        return cls(cls.MESH, mesh)

class PositionConstraint(Constraint):
    """Constrains a link to be within a certain position or volume.

    Attributes
    ----------
    link_name: string
        The name of the link this contraint refers to.
    bounding_volume: :class:`compas.geometry.BoundingVolume`
        The volume this constraint refers to.
    weight: float, optional
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important. Defaults to 1.
    """
    def __init__(self, link_name, bounding_volume, weight=1.):
        super(PositionConstraint, self).__init__(self.POSITION, weight)
        self.link_name = link_name
        self.bounding_volume = bounding_volume
        self.weight = weight


if __name__ == "__main__":
    from compas.datastructures import Mesh as CMesh
    import compas

    mesh = CMesh.from_obj(compas.get("faces.obj"))
    frame = Frame.worldXY()
    bv = BoundingVolume.from_box(Box(frame, 4, 4, 4))
    pc1 = PositionConstraint("link_name", bv)
    center, radius = (3,4,5), 10.
    bv = BoundingVolume.from_sphere(Sphere(center, radius))
    pc2 = PositionConstraint("link_name", bv)
    bv = BoundingVolume.from_mesh(mesh)
    pc3 = PositionConstraint("link_name", bv)

    oc = OrientationConstraint("link_name", frame.euler_angles(), tolerances=[0,0,0])
    
    jc = JointConstraint("joint_name", 1.4, 0.1)


    from compas_fab.backends.ros import Constraints
    from compas_fab.backends.ros import PositionConstraint as PositionConstraintROS
    from compas_fab.backends.ros import OrientationConstraint as OrientationConstraintROS
    from compas_fab.backends.ros import JointConstraint as JointConstraintROS
    from compas_fab.backends.ros import Header
    from compas_fab.backends.ros import Point, Quaternion, SolidPrimitive, Pose, Mesh

    goal_constraints = [pc1, pc2, pc3, oc, jc]
    goal_constraints = [pc1, pc2, oc, jc]

    C = Constraints()

    for c in goal_constraints:
        if c.type == Constraint.JOINT:
            print("JOINT")
            cr = JointConstraintROS(c.joint_name, c.value, c.tolerance, c.tolerance, c.weight)
            C.joint_constraints.append(cr)
        elif c.type == Constraint.POSITION:
            print("POSITION")
            cr = PositionConstraintROS(header=Header(), link_name=c.link_name)
            if c.bounding_volume.type == BoundingVolume.BOX:
                box = c.bounding_volume.volume
                bv = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[box.xsize, box.ysize, box.zsize])
                cr.constraint_region.primitives = [bv]
                cr.constraint_region.primitive_poses = [Pose.from_frame(box.frame)]
            elif c.bounding_volume.type == BoundingVolume.SPHERE:
                bv = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[c.bounding_volume.volume.radius])
                cr.constraint_region.primitives = [bv]
                cr.constraint_region.primitive_poses = [Pose(Point(*c.bounding_volume.volume.center), Quaternion(0, 0, 0, 1))]
            elif c.bounding_volume.type == BoundingVolume.MESH:
                cr.constraint_region.meshes = [Mesh.from_mesh(c.bounding_volume.volume)]
                cr.constraint_region.mesh_poses = [Pose()]            
            else:
                raise NotImplementedError
            C.position_constraints.append(cr)
        elif c.type == Constraint.ORIENTATION:
            print("ORIENTATION")
            cr = OrientationConstraintROS(header=Header(), link_name=c.link_name)
            cr.orientation = Quaternion.from_frame(Frame.from_euler_angles(c.euler_angles))
            ax, ay, az = Frame.from_euler_angles(c.tolerances).axis_angle_vector
            print(ax, ay, az)
            cr.absolute_x_axis_tolerance = ax
            cr.absolute_y_axis_tolerance = ay
            cr.absolute_z_axis_tolerance = az
            C.orientation_constraints.append(cr)
        else:
            raise NotImplementedError
    
    print(C)

    """
    pose = Pose.from_frame(frame)

        pcm = PositionConstraint(header=header, link_name=ee_link)
        pcm.target_point_offset.x = 0.
        pcm.target_point_offset.y = 0.
        pcm.target_point_offset.z = 0.
        bv = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[tolerance_position])
        pcm.constraint_region.primitives = [bv]
        pcm.constraint_region.primitive_poses = [Pose(pose.position, Quaternion(0, 0, 0, 1))]

        ocm = OrientationConstraint(header=header, link_name=ee_link)
        ocm.orientation = Quaternion.from_frame(frame)
        ocm.absolute_x_axis_tolerance = tolerance_angle
        ocm.absolute_y_axis_tolerance = tolerance_angle
        ocm.absolute_z_axis_tolerance = tolerance_angle
    """