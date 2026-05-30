# r: compas_fab>=1.1.0
"""
Build a Configuration from per-joint sliders auto-created from a RobotCell.

Wire a `robot_cell` to this component and one `GH_NumberSlider` per
configurable joint is generated automatically, with min/max set to the
joint's limits (and ±2π fallback for continuous joints / ±1m for prismatic
when no limit is declared). Each slider feeds a dedicated `j_<joint_name>`
input on the component, in canonical joint order.

Re-wiring a different `robot_cell` replaces the slider bank to match the
new joint signature.

For full manual control (no cell, custom joint names / types), use the
sibling `Cf_Configuration` component instead.

COMPAS FAB v1.1.0
"""

import Grasshopper
from compas_ghpython import error
from compas_robots import Configuration

from compas_fab.ghpython import ensure_joint_sliders


class RobotConfigurationComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, robot_cell):
        if robot_cell is None:
            return None

        try:
            robot_model = robot_cell.robot_model
        except AttributeError:
            error(ghenv.Component, "robot_cell must be a RobotCell instance.")  # noqa: F821
            return None

        joints = ensure_joint_sliders(ghenv.Component, robot_model)  # noqa: F821
        if not joints:
            return None

        names = [name for name, _, _ in joints]
        types = [jtype for _, jtype, _ in joints]
        values = [value for _, _, value in joints]
        return Configuration(joint_values=values, joint_types=types, joint_names=names)
