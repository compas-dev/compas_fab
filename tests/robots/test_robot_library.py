from compas.geometry import Point
from compas.geometry import Vector

from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import ToolLibrary
import pytest

# Every cell of the library, with the tool attached to its main planning group
CELLS_WITH_TOOLS = [
    ("ur5_cone_tool", "cone"),
    ("ur5_gripper_one_beam", "gripper"),
    ("ur10e_gripper_one_beam", "gripper"),
    ("abb_irb4600_40_255_gripper_one_beam", "gripper"),
    ("abb_irb4600_40_255_printing_tool", "printing_tool"),
]

# `printing_tool` is deliberately absent: it is an L-shaped tool whose tip sits a metre
# off to the side of its mount (it reaches along +Y of its base frame), which is how it
# has always been modelled. It still mounts by its base frame like every other tool.
TOOLS_REACHING_ALONG_Z = ["cone", "static_gripper", "static_gripper_small", "kinematic_gripper"]


@pytest.fixture
def all_robots_cells_and_states():
    robots = []
    robots.append(RobotCellLibrary.rfl())
    robots.append(RobotCellLibrary.ur3())
    robots.append(RobotCellLibrary.ur3e())
    robots.append(RobotCellLibrary.ur5())
    robots.append(RobotCellLibrary.ur5e())
    robots.append(RobotCellLibrary.ur10())
    robots.append(RobotCellLibrary.ur10e())
    robots.append(RobotCellLibrary.ur16e())
    robots.append(RobotCellLibrary.panda())
    robots.append(RobotCellLibrary.staubli_tx2_60l())
    robots.append(RobotCellLibrary.abb_irb4600_40_255())
    return robots


def test_robot_semantics_and_geometry(all_robots_cells_and_states):
    for robot_cell, robot_state in all_robots_cells_and_states:
        robot_cell.ensure_semantics()
        robot_cell.ensure_geometry()
        assert robot_cell.robot_model.name
        robot_cell.print_info()


@pytest.mark.parametrize("tool_name", TOOLS_REACHING_ALONG_Z)
def test_tools_mount_along_their_z_axis(tool_name):
    """The mounting convention of the library: a tool reaches along the +Z axis of
    its base frame, which is the frame the robot's flange takes hold of it at.

    This is what lets the same tool be attached to any robot of the library without
    a rotation, and it is only true because every planning group there ends at a
    link whose +Z points away from the arm.
    """
    tool = getattr(ToolLibrary, tool_name)(load_geometry=True)

    tcf_direction = Vector(*tool.frame.point).unitized()
    assert tcf_direction.dot(Vector.Zaxis()) > 0.9, "{} does not reach along +Z".format(tool_name)

    # ...and the TCF states the working direction with its own Z axis, which is the axis
    # a TargetMode.TOOL target aligns the tool to. A TCF left pointing along X mounts the
    # tool correctly but sends every tool-mode target off at 90 degrees.
    assert Vector(*tool.frame.zaxis).dot(Vector.Zaxis()) > 0.99, "{} works along its TCF X, not its TCF Z".format(tool_name)

    # The body of the tool sits on the +Z side of its mounting plane, i.e. it does not
    # reach back into the robot it is attached to. Only the base link is checked, the
    # geometry of any child link is expressed in that link's own frame, not this one.
    for mesh in tool.get_link_collision_meshes(tool.root) or []:
        zs = [mesh.vertex_coordinates(v)[2] for v in mesh.vertices()]
        assert min(zs) > -1e-6, "{} has geometry behind its mounting plane".format(tool_name)


@pytest.mark.parametrize("cell_name, tool_id", CELLS_WITH_TOOLS)
def test_cells_attach_their_tool_without_a_rotation(cell_name, tool_id):
    """Because tools and end effector links agree on the convention, every cell of
    the library attaches its tool with an identity attachment frame, on UR and ABB
    alike. A rotation creeping back in means the two conventions have drifted apart.
    """
    robot_cell, robot_cell_state = getattr(RobotCellLibrary, cell_name)(load_geometry=False)
    attachment_frame = robot_cell_state.tool_states[tool_id].attachment_frame

    assert attachment_frame.point.distance_to_point(Point(0, 0, 0)) == pytest.approx(0.0, abs=1e-9)
    assert Vector(*attachment_frame.xaxis).dot(Vector.Xaxis()) == pytest.approx(1.0, abs=1e-9)
    assert Vector(*attachment_frame.yaxis).dot(Vector.Yaxis()) == pytest.approx(1.0, abs=1e-9)


@pytest.mark.parametrize("cell_name, tool_id", CELLS_WITH_TOOLS)
def test_attached_tool_sits_where_the_tool_says_it_does(cell_name, tool_id):
    """With the conventions agreeing, the TCP relative to the end effector link is
    simply the tool's own TCF — no rotation, no offset introduced by the attachment.

    This is the pose the scene objects draw, `RobotCell.compute_attach_objects_frames`
    is what `BaseRobotCellObject.update` calls, so this is the "tool looks wrong in
    Rhino" regression.
    """
    robot_cell, robot_cell_state = getattr(RobotCellLibrary, cell_name)(load_geometry=False)
    robot_cell_state.robot_configuration.joint_values = [-2.238, -1.153, -2.174, 0.185, 0.667, 0.5]

    tcf_in_pcf = Point(0, 0, 0).transformed(robot_cell.t_pcf_tcf(robot_cell_state, tool_id))
    assert tcf_in_pcf.distance_to_point(Point(*robot_cell.tool_models[tool_id].frame.point)) < 1e-9

    # ...and in the world it lands on the frame of the end effector link
    ee_frame = robot_cell.robot_model.forward_kinematics(robot_cell_state.robot_configuration, robot_cell.get_end_effector_link_name())
    assert (
        ee_frame.to_world_coordinates(tcf_in_pcf).distance_to_point(
            robot_cell.compute_attach_objects_frames(robot_cell_state).tool_states[tool_id].frame.to_world_coordinates(Point(*robot_cell.tool_models[tool_id].frame.point))
        )
        < 1e-9
    )
