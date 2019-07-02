"""Creates an AssemblyNetwork and creates a problem instance pkg for stripstream

    Inputs
    ------
    network_lines: a list of rg.Curve
    network_vertices: a list of rg.Point3d
        this list must be indexed correponding to the tree branch in
        assembly objects
    assembly_objects: a DataTree of rg.Mesh

    assembly_object_poses:

    world_from_grasp_pose
    Outputs
    -------
        a: The a output variable
"""

import sys
import copy
import os
import json
from collections import OrderedDict

import Rhino.Geometry as rg
import ghpythonlib.components as ghcomp
import ghpythonlib.treehelpers as th

from compas.datastructures import Mesh, Network
from compas.geometry import Frame, Point
from compas.geometry import Transformation, transform_points, cross_vectors
from compas_ghpython import xdraw_mesh, xdraw_frame

##########################################
def tree_to_lists(input):
    """converts a depth-2 GH datatree to a list of lists
    """
    all = []
    for path in input.Paths:
        branch_list = []
        for obj in input.Branch(path):
            branch_list.append(obj)
        all.append(branch_list)
    return all

def rPln2cFrame(rplane):
    """converts a rhino plane to compas frame
    """
    return Frame(rplane.Origin, rplane.XAxis, rplane.YAxis)

def transform_cmesh(cmesh, ctf):
    """transform a compas mesh
    Paramters
    ---------
    cmesh: a compas mesh
    ctf: a compas transformation
    """
    verts, faces = cmesh.to_vertices_and_faces()
    for i, v in enumerate(verts):
        v_tf = transform_points([v], ctf)[0]
        cmesh.set_vertex_attributes(i, ['x','y','z'], v_tf)
    return cmesh

def network_from_rg_pts_and_lines(vertices, lines, tol=1e-3):
    """create a compas network using given rhino pts and lines

    Paramters
    ---------
    vertices: list of rh.point3d
    edges: list of rh.curve

    Returns
    -------
    """
    net_lines = []
    for crv in lines:
        rg_end_pt_1, rg_end_pt_2 = ghcomp.EndPoints(crv)
        end_1_id = -1
        end_2_id = -1

        # convert rhino pt to compas pt
        for v_id, v in enumerate(vertices):
            if rg_end_pt_1.DistanceTo(v) < tol:
                end_1_id = v_id
            if rg_end_pt_2.DistanceTo(v) < tol:
                end_2_id = v_id

        assert((end_1_id != -1) \
        and (end_2_id != -1) \
        and (end_1_id != end_2_id))

        net_lines.append((end_1_id, end_2_id))

    cp_verts = [rPt2cPt(rpt) for rpt in vertices]
    return Network.from_vertices_and_edges(cp_verts, net_lines)

def draw_assembly_object_cmeshes(as_o, stage="place"):
    """
    Parameters:
        as_o: AssemblyObject
        stage: flag for 'place', 'pick'
    """
    if stage == "place":
        return [xdraw_mesh(*cm.to_vertices_and_faces()) for cm in as_o.shape_in_place_pose]
    elif "pick":
        return [xdraw_mesh(*cm.to_vertices_and_faces()) for cm in as_o.shape_in_pick_pose]
    else:
        return None

# -----------------
# json export utils
# -----------------
def obj_name(obj_id, sub_id):
    name = "object_" + str(obj_id) + "-" + str(sub_id) + ".obj"
    return name

def cframe2json(cfr):
    """
    Parameters
    ----------
    cfr: compas frame
    """
    dict = OrderedDict()
    dict["XAxis"] = cfr.to_data()["xaxis"]
    dict["YAxis"] = cfr.to_data()["yaxis"]
    dict["ZAxis"] = cross_vectors(dict["XAxis"], dict["YAxis"])
    dict["Origin"] = cfr.to_data()["point"]
    return dict

##########################################
WORLD_FRAME = Frame.worldXY()

class AssemblyObject(object):
    """assembly object that embodies geometry, geometry pose, grasp pose

    Attributes
    ----------
    base_frame: compas.geometry.Frame
        The frame that the data is associated with/described in.
        Defaults to be the 'base_link' (world xyz).
    id: string
        The id of the object, usually an int index of the object
    shape: compas.geometry.Mesh
        The assembly object's geometry, the mesh vertices coordinates are
        associated to the 'base_link' (world xyz)
    shape_pick_pose: compas.geometry.Frame
        The shape's pose to be picked described in the associated base_frame
    shape_place_pose: compas.geometry.Frame
        The shape's pose to be placed described in the associated base_frame
    shape_time_stamped_pose: dict{float : compas.geometry.Frame}
        current shape pose
    object_from_ee_poses: list of compas.geometry.Frame
        The end effector (ee/gripper) grasp poses described in the object's
        shape_pose frame.
    """

    def __init__(self, base_frame=WORLD_FRAME, id=-1, \
        shape=None, shape_place_pose=None):
        """

        Parameters
        ----------
            base_frame: compas.geometry.Frame
                Defaults to the world xyz
            id: int
                Defaults to be -1
            shape: compas.geometry.Mesh
                Defaults to be None
        """
        self.base_frame = base_frame
        self.id = id
        self.shape = shape
        self.shape_place_pose = None
        self.shape_pick_pose = None

    # --------------
    # constructors
    # --------------
    @classmethod
    def create_from_object_place_pose(cls, id, rbase_frame, \
            rgeometry_in_scene, rpose_in_scene):
        """create object from geometry in a scene pose

        We perform transformation here to orient the geometry back to
        the base_link

        Parameters
        ----------
        rbase_frame : rhion plane
        rgeometry_in_scene: list of rhino meshes
        rpose_in_scene: rhino planes
        """
        as_obj = cls()
        as_obj.base_frame = rPln2cFrame(rbase_frame)
        as_obj.shape_place_pose = rPln2cFrame(rpose_in_scene)

        # perform transformation on the geometry_in_scene
        obj_tf = Transformation.from_frame(as_obj.shape_place_pose)

        cmesh_tf = [transform_cmesh(rMesh2cMesh(rm), obj_tf.inverse()) \
                for rm in rgeometry_in_scene]

        as_obj.shape = cmesh_tf

        return as_obj

    # --------------
    # attributes assigners
    # --------------
    def set_pick_in_scene_pose(self, rpick_plane):
        self.shape_pick_pose = rPln2cFrame(rpick_plane)

    def set_place_in_scene_pose(self, rplace_plane):
        self.shape_place_pose = rPln2cFrame(rplace_plane)

    def set_ee_poses_from_in_scene_poses(self, rworld_from_obj_pose, rworld_from_ee_poses):
        """create obj_from_grasp poses from world

        Parameters
        ----------
        rworld_from_obj_pose: rhino plane
        rworld_from_ee_poses: a list of rhino plane
        """
        world_obj_tf = Transformation.from_frame( \
            rPln2cFrame(rworld_from_obj_pose))

        self.object_from_ee_poses = []
        for world_ee_fr in [rPln2cFrame(rw_ee) for rw_ee in rworld_from_ee_poses]:
            w_ee_tf = Transformation.from_frame(world_ee_fr)
            self.object_from_ee_poses.append(Frame.from_transformation( \
                Transformation.concatenate(world_obj_tf.inverse(), w_ee_tf)))


    def set_ee_place_poses_from_obj(self, obj_from_ee_poses):
        assert(False, "not implemented")

    @property
    def world_from_pick_ee_poses(self):
        """return a list of (world_from) ee pick poses
        """
        assert(self.shape_pick_pose != None, "pick pose not defined!")
        world_from_ee_pick = []
        world_pick_tf = Transformation.from_frame(self.shape_pick_pose)
        for obj_from_ee in self.object_from_ee_poses:
            w_f_ee_tf = Transformation.concatenate(world_pick_tf, Transformation.from_frame(obj_from_ee))
            world_from_ee_pick.append(Frame.from_transformation(w_f_ee_tf))
        return world_from_ee_pick

    @property
    def world_from_place_ee_poses(self):
        """return a list of (world_from) ee place poses
        """
        assert(self.shape_place_pose != None, "place pose not defined!")
        world_from_ee_place = []
        world_place_tf = Transformation.from_frame(self.shape_place_pose)
        for obj_from_ee in self.object_from_ee_poses:
            w_f_ee_tf = Transformation.concatenate(world_place_tf, Transformation.from_frame(obj_from_ee))
            world_from_ee_place.append(Frame.from_transformation(w_f_ee_tf))
        return world_from_ee_place

    @property
    def shape_in_pick_pose(self):
        """return shape geometries in pick pose"""
        assert(self.shape_pick_pose != None, "pick pose not defined!")
        world_pick_tf = Transformation.from_frame(self.shape_pick_pose)
        return [transform_cmesh(cm, world_pick_tf) for cm in self.shape]
#        return [cm for cm in self.shape]

    @property
    def shape_in_place_pose(self):
        """return shape geometries in pick pose"""
        assert(self.shape_place_pose != None, "place pose not defined!")
        world_place_tf = Transformation.from_frame(self.shape_place_pose)
        return [transform_cmesh(cm, world_place_tf) for cm in self.shape]

class AssemblyNetwork(Network):
    def __init__(self):
        super(AssemblyNetwork, self).__init__()

    
    # --------------
    # properties getters
    # --------------
    @property
    def assembly_objects(self):
        return [self.get_vertex_attribute(v, "assembly_object") \
            for v in self.vertices()]

    def assembly_object(self, vert_id):
        return self.assembly_objects[vert_id]

    # --------------
    # exporters
    # --------------
    def save_assembly_object_to_objs(self, mesh_path):
        if not os.path.isdir(mesh_path):
            os.mkdir(mesh_path)

        for as_obj in self.assembly_objects:
            for i, s in enumerate(as_obj.shape):
                s.to_obj(os.path.join(mesh_path, obj_name(as_obj.id, i)))

    def save_assembly_network_to_json(self, json_path, pkg_name='', \
            assembly_type='', model_type='', unit=''):
        if not os.path.isdir(json_path):
            os.mkdir(json_path)

        json_file_path = os.path.join(json_path, pkg_name + '.json')

        data = OrderedDict()
        data['pkg_name'] = pkg_name
        data['assembly_type'] = assembly_type
        data['model_type'] = model_type
        data['unit'] = unit

        data['element_number'] = self.number_of_vertices()
        data['sequenced_elements'] = []

        for v in self.vertices():
            assert(v == self.assembly_object(v).id)
            ao = self.assembly_object(v)

            e_data = OrderedDict()
            e_data['order_id'] = v
            e_data['base_frame'] = cframe2json(ao.base_frame)
            e_data['element_geometry_file_names'] = [obj_name(v, sub_id) \
                for sub_id in range(len(ao.shape))]

            grasp_data = OrderedDict()
            grasp_data['parent_link'] = 'object' #world
            grasp_data['ee_poses'] = [cframe2json(ee_p) \
                for ee_p in ao.object_from_ee_poses]

            pick = OrderedDict()
            pick['process_name'] = 'pick'
            pick['base_frame'] = cframe2json(ao.base_frame)
            pick['target_pose'] = cframe2json(ao.shape_pick_pose)
            pick['allowed_collision_obj_names'] = [] # support tables

            place = OrderedDict()
            place['process_name'] = 'place'
            place['base_frame'] = cframe2json(ao.base_frame)
            place['target_pose'] = cframe2json(ao.shape_place_pose)
            place['allowed_collision_obj_names'] = [] # support tables
            for nghb_id in self.vertex_neighborhood(v):
                place['allowed_collision_obj_names'].extend(
                    [obj_name(nghb_id, sub_id) \
                        for sub_id in range(len(self.assembly_object(nghb_id).shape))])

            grasp_data['processes'] = [pick, place]
            e_data['grasps'] = grasp_data

            data['sequenced_elements'].append(e_data)

        with open(json_file_path, 'w') as outfile:
            json.dump(data, outfile, indent=4)

    def save_assembly_network_to_urdf(self, urdf_path):
        pass

    def save_to_stripstream_pkg(self, save_path, pkg_name, \
        assembly_type="", model_type="", unit=""):
        root_path = os.path.join(save_path, pkg_name)
        if not os.path.isdir(root_path):
            os.mkdir(root_path)

        json_path = os.path.join(root_path, "json")
        mesh_path = os.path.join(root_path, "meshes", "collision")
        urdf_path = os.path.join(root_path, "urdf")
        check_paths = [json_path, mesh_path, urdf_path]
        for p in check_paths:
            if not os.path.isdir(p):
                os.mkdir(p)

        # generate obj files
        self.save_assembly_object_to_objs(mesh_path)

        # generate json
        self.save_assembly_network_to_json(json_path, pkg_name, assembly_type, model_type, unit)

        # genereate urdf
#        self.save_assembly_network_to_urdf(urdf_path)

########################################
# main
a = []
b = []
c = []

# temp drawing
tf_meshes = []

line_net = network_from_rg_pts_and_lines(network_vertices, network_lines)
#a = get_network_rhino_axes(line_net)

# create assembly objects
obj_lists = tree_to_lists(assembly_objects)
grasp_pose_lists = tree_to_lists(world_from_grasp_poses)
as_objs = []

for i, obj_list in enumerate(obj_lists):
    as_obj = AssemblyObject.create_from_object_place_pose( \
        str(i), base_frame,
        obj_list, world_from_place_poses[i])
    as_obj.set_pick_in_scene_pose(world_from_pick_poses[i])
    as_obj.set_ee_poses_from_in_scene_poses(world_from_place_poses[i], grasp_pose_lists[i])

    as_objs.append(as_obj)

as_o = as_objs[0]
#print(as_o.shape_pick_pose.to_data())
#a = xdraw_frame(as_o.shape_pick_pose)
#b = [xdraw_mesh(*cm.to_vertices_and_faces()) for cm in as_o.shape_in_pick_pose]
#c = [xdraw_frame(ee) for ee in as_o.world_from_pick_ee_poses]

assembly_net = AssemblyNetwork.from_network_and_assembly_objects(line_net, as_objs)

if generate:
    assembly_net.save_to_stripstream_pkg(file_path, pkg_name, assembly_type, model_type, unit)

## inspect vertices in neighborhood
#v_id = 1
#v_nbhd = assembly_net.vertex_neighborhood(v_id)
#for v in v_nbhd:
#    as_o_nbgh = assembly_net.get_vertex_attribute(v, "assembly_object")
#    a.extend(draw_assembly_object_cmeshes(as_o_nbgh))
#
#b = draw_assembly_object_cmeshes(assembly_net.get_vertex_attribute(v_id, "assembly_object"))
