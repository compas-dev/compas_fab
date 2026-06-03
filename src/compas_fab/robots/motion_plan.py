"""Higher-level container for assembling planned motions into a named, serializable workflow."""

import hashlib
from copy import deepcopy
from typing import TYPE_CHECKING
from typing import Any
from typing import Iterator
from typing import Optional
from typing import cast

from compas.data import Data

from compas_fab.robots.trajectory import JointTrajectory

if TYPE_CHECKING:
    from compas_fab.robots import RobotCell
    from compas_fab.robots import RobotCellState

__all__ = [
    "MotionPlan",
    "PlanStep",
]


def _cell_signature(cell: "RobotCell") -> str:
    """Stable fingerprint of a cell's structural identity (robot name + tool/body ids)."""
    parts = [cell.robot_model.name or "", *sorted(cell.tool_ids), *sorted(cell.rigid_body_ids)]
    return hashlib.sha256("|".join(parts).encode("utf-8")).hexdigest()


class PlanStep:
    """A single step within a [MotionPlan][compas_fab.robots.MotionPlan].

    A step is one of two kinds:

    - **Trajectory step** — carries a [JointTrajectory][compas_fab.robots.JointTrajectory].
      The plan derives the post-state by applying the trajectory's last point
      to the predecessor's state.
    - **State-change step** — carries no trajectory. The post-state is explicit
      (typically: a tool was attached/detached, a rigid body grasped/released).

    Construction is normally done through `MotionPlan.append_trajectory()` and
    `MotionPlan.append_state_change()`, which take care of validation and
    post-state derivation.

    Parameters
    ----------
    name
        Unique identifier of the step within the plan.
    trajectory
        The trajectory for a trajectory step. `None` for a state-change step.
    post_state
        Cell state after the step has been executed. For trajectory steps the
        plan derives this; for state-change steps it must be supplied.
    description
        Free-form human description.

    Attributes
    ----------
    is_trajectory
        True when this step carries a trajectory.
    duration
        Trajectory duration in seconds, or `0.0` for state-change steps.
    """

    def __init__(
        self,
        name: str,
        trajectory: Optional[JointTrajectory] = None,
        post_state: Optional["RobotCellState"] = None,
        description: str = "",
    ):
        self.name = name
        self.trajectory = trajectory
        self.post_state = post_state
        self.description = description

    @property
    def is_trajectory(self) -> bool:
        return self.trajectory is not None

    @property
    def duration(self) -> float:
        return self.trajectory.time_from_start if self.trajectory is not None else 0.0


class MotionPlan(Data):
    """A named, serializable sequence of trajectories and state changes.

    A plan threads a single [RobotCellState][compas_fab.robots.RobotCellState]
    through a chain of steps. The post-state of step N is the pre-state of
    step N+1. Trajectory steps derive their post-state automatically (the
    trajectory's last point applied to the pre-state); state-change steps
    (e.g. gripper toggle, tool attach/detach) carry an explicit post-state.

    The typical use case is pick-and-place: approach → contact → state-change
    (grasp) → retract → transfer → approach → contact → state-change (release)
    → retract. Each motion is planned separately and atomically; the plan is
    the higher-level assembly.

    Iteration yields steps along the realized path. A `step_by_name()` lookup
    is provided; index-based access is intentionally not exposed so that
    future extensions (e.g. branching alternatives) do not require an API
    break.

    Parameters
    ----------
    name
        Plan identifier.
    start_state
        Cell state the plan starts from.
    robot_cell
        When provided, a structural signature of the cell is stored so that
        loading the plan against a mismatched cell can fail loudly (see
        `verify_cell()`).
    description
        Free-form human description of the plan.

    Attributes
    ----------
    end_state
        Post-state of the last step, or `start_state` if the plan is empty.
    trajectories
        The trajectories along the realized path, in order.
    duration
        Sum of trajectory durations along the realized path, in seconds.
    cell_signature
        Fingerprint of the bound cell, or `None` if the plan was built without
        one.

    Notes
    -----
    On serialization the plan strips the embedded `start_state` from each
    contained trajectory (it is derivable from the chain) and omits the
    derived `post_state` of trajectory steps. Only explicit state-change
    post-states are stored.
    """

    def __init__(
        self,
        name: str,
        start_state: "RobotCellState",
        robot_cell: Optional["RobotCell"] = None,
        description: str = "",
    ):
        super().__init__()
        self.name = name
        self.description = description
        self.start_state = start_state
        self.cell_signature: Optional[str] = _cell_signature(robot_cell) if robot_cell is not None else None
        self._steps: list[PlanStep] = []

    def __iter__(self) -> Iterator[PlanStep]:
        return iter(self._steps)

    def __len__(self) -> int:
        return len(self._steps)

    def iter_steps(self) -> Iterator[PlanStep]:
        """Iterate over steps along the realized path."""
        return iter(self._steps)

    @property
    def steps(self) -> list[PlanStep]:
        """Snapshot of the realized path. Returns a fresh list; mutating it does not affect the plan."""
        return list(self._steps)

    def step_by_name(self, name: str) -> PlanStep:
        """Return the step with the given name.

        Raises
        ------
        KeyError
            If no step with that name exists.
        """
        for s in self._steps:
            if s.name == name:
                return s
        raise KeyError("No step named {!r} in plan {!r}".format(name, self.name))

    @property
    def end_state(self) -> "RobotCellState":
        if not self._steps:
            return self.start_state
        return cast("RobotCellState", self._steps[-1].post_state)

    @property
    def trajectories(self) -> list[JointTrajectory]:
        return [s.trajectory for s in self._steps if s.trajectory is not None]

    @property
    def duration(self) -> float:
        return sum(s.duration for s in self._steps)

    def append_trajectory(
        self,
        name: str,
        trajectory: JointTrajectory,
        description: str = "",
    ) -> "MotionPlan":
        """Append a trajectory step.

        The post-state is derived from the current `end_state` with the
        trajectory's last point applied. The trajectory's joint set must be a
        subset of the cell's configurable joints.

        Returns `self` to allow chaining.

        Raises
        ------
        ValueError
            If `name` is already used, or the trajectory references joints
            absent from the cell state.
        """
        self._assert_name_unused(name)
        pre_state = self.end_state
        self._validate_trajectory(trajectory, pre_state)
        post_state = self._derive_post_state(pre_state, trajectory)
        self._steps.append(PlanStep(name=name, trajectory=trajectory, post_state=post_state, description=description))
        return self

    def append_state_change(
        self,
        name: str,
        post_state: "RobotCellState",
        description: str = "",
    ) -> "MotionPlan":
        """Append a discrete state-change step (e.g. attach/detach tool, grasp/release).

        Returns `self` to allow chaining.

        Raises
        ------
        ValueError
            If `name` is already used.
        """
        self._assert_name_unused(name)
        self._steps.append(PlanStep(name=name, post_state=post_state, description=description))
        return self

    def append_step(self, step: PlanStep) -> "MotionPlan":
        """Append a pre-built [PlanStep][compas_fab.robots.PlanStep].

        Dispatches to `append_trajectory()` or `append_state_change()`
        depending on the step's kind. Convenient when steps are constructed
        upstream (e.g. by Grasshopper step-builder components) and the
        caller just needs to thread them in order.

        Raises
        ------
        ValueError
            If `step.name` is already used, the trajectory references joints
            absent from the cell state, or a state-change step is missing
            its `post_state`.
        """
        if step.is_trajectory:
            return self.append_trajectory(step.name, step.trajectory, step.description)
        if step.post_state is None:
            raise ValueError("State-change step {!r} requires an explicit post_state".format(step.name))
        return self.append_state_change(step.name, step.post_state, step.description)

    def verify_cell(self, robot_cell: "RobotCell") -> None:
        """Verify the plan was assembled against `robot_cell`.

        Compares the cached cell signature against the given cell.

        Raises
        ------
        ValueError
            If the signatures differ. No-op when the plan was constructed
            without a bound cell.
        """
        if self.cell_signature is None:
            return
        actual = _cell_signature(robot_cell)
        if actual != self.cell_signature:
            raise ValueError(
                "Plan {!r} was assembled against a different cell. "
                "Expected signature {}..., got {}...".format(self.name, self.cell_signature[:16], actual[:16])
            )

    def _assert_name_unused(self, name: str) -> None:
        if any(s.name == name for s in self._steps):
            raise ValueError("Step name {!r} already used in plan {!r}".format(name, self.name))

    def _validate_trajectory(self, trajectory: JointTrajectory, state: "RobotCellState") -> None:
        if state.robot_configuration is None:
            return
        cell_joints = set(state.robot_configuration.joint_names or [])
        if not cell_joints:
            return
        traj_joints = set(trajectory.joint_names or [])
        if traj_joints and not traj_joints.issubset(cell_joints):
            missing = sorted(traj_joints - cell_joints)
            raise ValueError(
                "Trajectory references joints {} that are not present in the cell state".format(missing)
            )

    @staticmethod
    def _derive_post_state(pre_state: "RobotCellState", trajectory: JointTrajectory) -> "RobotCellState":
        new_state = deepcopy(pre_state)
        if not trajectory.points:
            return new_state
        last = trajectory.points[-1]
        joint_names = list(last.joint_names) or list(trajectory.joint_names or [])
        base = new_state.robot_configuration
        if base is None or not joint_names:
            new_state.robot_configuration = last
            return new_state
        name_to_pos = {n: i for i, n in enumerate(base.joint_names)}
        new_values = list(base.joint_values)
        for n, v in zip(joint_names, last.joint_values):
            idx = name_to_pos.get(n)
            if idx is not None:
                new_values[idx] = v
        base.joint_values = new_values
        return new_state

    @property
    def __data__(self) -> dict[str, Any]:
        from compas_fab.robots import RobotCellState  # noqa: F401  -- for clarity at read-time

        steps_data = []
        for step in self._steps:
            d: dict[str, Any] = {"name": step.name, "description": step.description}
            if step.trajectory is not None:
                td = step.trajectory.__data__
                td["start_state"] = None  # stripped — derivable from chain
                d["trajectory"] = td
            else:
                d["post_state"] = step.post_state.__data__ if step.post_state is not None else None
            steps_data.append(d)
        return {
            "name": self.name,
            "description": self.description,
            "start_state": self.start_state.__data__,
            "cell_signature": self.cell_signature,
            "steps": steps_data,
        }

    @classmethod
    def __from_data__(cls, data: dict[str, Any]) -> "MotionPlan":
        from compas_fab.robots import RobotCellState

        start_state = cast("RobotCellState", RobotCellState.__from_data__(data["start_state"]))
        plan = cls(name=data["name"], start_state=start_state, description=data.get("description", ""))
        plan.cell_signature = data.get("cell_signature")

        running_state = start_state
        for sd in data["steps"]:
            if "trajectory" in sd:
                traj = JointTrajectory.__from_data__(sd["trajectory"])
                traj.start_state = running_state
                plan.append_trajectory(sd["name"], traj, description=sd.get("description", ""))
                running_state = cast("RobotCellState", plan._steps[-1].post_state)
            else:
                post_state = cast("RobotCellState", RobotCellState.__from_data__(sd["post_state"]))
                plan.append_state_change(sd["name"], post_state, description=sd.get("description", ""))
                running_state = post_state
        return plan
