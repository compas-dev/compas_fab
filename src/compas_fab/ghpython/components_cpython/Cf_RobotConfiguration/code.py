# r: compas_fab>=1.1.0
"""
Build a Configuration from per-joint sliders auto-created from a RobotCell.

Wire a `robot_cell` to this component and one `GH_NumberSlider` per
configurable joint is generated automatically and wired to the `joints`
list input, with min/max set to the joint's limits (and ±2π fallback for
continuous joints / ±1m for prismatic when no limit is declared).

Re-wiring a different `robot_cell` replaces the slider bank to match the
new joint signature. Wiring your own values to `joints` overrides the
auto-sliders.

COMPAS FAB v1.1.0
"""

import Grasshopper
from compas_ghpython import error
from compas_robots import Configuration

from compas_fab.ghpython import ensure_joint_sliders


class RobotConfigurationComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, robot_cell, joints):
        if robot_cell is None:
            return None

        try:
            robot_model = robot_cell.robot_model
        except AttributeError:
            error(ghenv.Component, "robot_cell must be a RobotCell instance.")  # noqa: F821
            return None

        metadata = ensure_joint_sliders(ghenv.Component, robot_model, input_name="joints")  # noqa: F821
        if not metadata:
            return None

        names = [name for name, _, _ in metadata]
        types = [jtype for _, jtype, _ in metadata]
        defaults = [default for _, _, default in metadata]

        # `joints` may arrive empty on the very first solve (sliders are
        # wired in this pass but their values only flow on the scheduled
        # follow-up solve). Fall back to per-joint defaults then.
        if not joints or len(joints) < len(metadata):
            values = defaults
        else:
            values = [float(v) for v in joints[: len(metadata)]]

        return Configuration(joint_values=values, joint_types=types, joint_names=names)
