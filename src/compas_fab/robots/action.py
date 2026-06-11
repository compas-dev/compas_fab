"""Higher-level container for assembling planned motions into a named, serializable workflow."""

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
    "Action",
    "ActionChain",
]


class Action:
    """A single action within an [ActionChain][compas_fab.robots.ActionChain].

    An action is the atomic unit of a chain. It wraps a `start_state` (the cell
    state the action begins from), an optional `trajectory`, an optional
    `post_state`, and a free-form `attributes` bag (with `tags` as the default,
    first-class attribute). It deliberately uses **one** class rather than a
    planned/unplanned split:

    - **Has a trajectory** → a *planned* action. The `post_state` is derived by
      applying the trajectory's last point to `start_state` (the chain does this
      on append).
    - **No trajectory** → an *unplanned* action. For a discrete state change
      (e.g. a tool attached/detached, a rigid body grasped/released) the
      `post_state` is supplied explicitly.

    Using a single class means backtracking/undo just clears `trajectory`
    instead of swapping class types. Branching is intentionally **not**
    implemented yet, but nothing here forecloses it.

    Construction is normally done through `ActionChain.append_trajectory()` and
    `ActionChain.append_state_change()`, which take care of validation,
    start-state threading and post-state derivation. Actions can also be built
    standalone (e.g. by Grasshopper builder components) and threaded in later
    with `ActionChain.append_action()`; in that case `start_state` stays `None`
    until the action is appended to a chain.

    Parameters
    ----------
    name
        Unique identifier of the action within the chain.
    start_state
        Cell state the action begins from. Filled in by the chain on append
        when the action is built standalone.
    trajectory
        The trajectory for a planned action. `None` for a state-change action.
    post_state
        Cell state after the action has executed. For planned actions the chain
        derives this; for state-change actions it must be supplied.
    description
        Free-form human description.
    tags
        Convenience accessor for `attributes["tags"]`, a list of strings driving
        differentiated downstream execution (e.g. `approach`, `retract`,
        `linear`, `free`). Stored in `attributes`.
    attributes
        Free-form key-value metadata. `tags` is the default key; execution code
        is free to read any other key (e.g. an execution speed or blend mode).

    Attributes
    ----------
    is_trajectory
        True when this action carries a trajectory (a *planned* action).
    duration
        Trajectory duration in seconds, or `0.0` for state-change actions.
    end_state
        Alias of `post_state`; the cell state once the action has executed.
    """

    def __init__(
        self,
        name: str,
        start_state: Optional["RobotCellState"] = None,
        trajectory: Optional[JointTrajectory] = None,
        post_state: Optional["RobotCellState"] = None,
        description: str = "",
        tags: Optional[list[str]] = None,
        attributes: Optional[dict[str, Any]] = None,
    ):
        self.name = name
        self.start_state = start_state
        self.trajectory = trajectory
        self.post_state = post_state
        self.description = description
        self.attributes: dict[str, Any] = dict(attributes) if attributes else {}
        if tags is not None:
            self.attributes["tags"] = list(tags)
        self.attributes.setdefault("tags", [])

    @property
    def tags(self) -> list[str]:
        """Differentiated-execution tags; convenience accessor over `attributes["tags"]`."""
        return self.attributes.setdefault("tags", [])

    @tags.setter
    def tags(self, value: Optional[list[str]]) -> None:
        self.attributes["tags"] = list(value or [])

    @property
    def is_trajectory(self) -> bool:
        return self.trajectory is not None

    @property
    def is_planned(self) -> bool:
        """True when the action carries a trajectory. Alias of `is_trajectory`, in the planned/unplanned framing."""
        return self.trajectory is not None

    @property
    def duration(self) -> float:
        return self.trajectory.time_from_start if self.trajectory is not None else 0.0

    @property
    def end_state(self) -> Optional["RobotCellState"]:
        return self.post_state


class ActionChain(Data):
    """A named, serializable sequence of trajectory and state-change actions.

    A chain threads a single [RobotCellState][compas_fab.robots.RobotCellState]
    through a sequence of [Action][compas_fab.robots.Action] objects. The
    post-state of action N is the pre-state (start_state) of action N+1 — this
    end-state-equals-next-start-state invariant is exactly why it is a *chain*.
    Planned actions derive their post-state automatically (the trajectory's last
    point applied to the pre-state); state-change actions (e.g. gripper toggle,
    tool attach/detach) carry an explicit post-state.

    The typical use case is pick-and-place: approach → contact → state-change
    (grasp) → retract → transfer → approach → contact → state-change (release)
    → retract. Each motion is planned separately and atomically; the chain is
    the higher-level assembly.

    Iteration yields actions along the realized path. An `action_by_name()`
    lookup is provided; index-based access is intentionally not exposed so that
    future extensions (e.g. branching alternatives — deliberately deferred for
    now) do not require an API break.

    The name "chain" was chosen because it conveys the end-state-equals-next-
    start-state link between actions. Rejected alternatives: *motion skeleton*,
    *template*, *motion plan*, *action sequence*, *action group*.

    Parameters
    ----------
    name
        Chain identifier.
    start_state
        Cell state the chain starts from.
    robot_cell
        When provided, a structural signature of the cell is stored so that
        loading the chain against a mismatched cell can fail loudly (see
        `verify_cell()`).
    description
        Free-form human description of the chain.

    Attributes
    ----------
    end_state
        Post-state of the last action, or `start_state` if the chain is empty.
    trajectories
        The trajectories along the realized path, in order.
    duration
        Sum of trajectory durations along the realized path, in seconds.
    cell_signature
        Fingerprint of the bound cell, or `None` if the chain was built without
        one.

    Notes
    -----
    On serialization the chain strips the embedded `start_state` from each
    contained trajectory and from each action (both are derivable by re-threading
    the chain) and omits the derived `post_state` of planned actions. Only
    explicit state-change post-states are stored. On load the chain is re-threaded
    so every action's `start_state` and every trajectory's mirrored `start_state`
    are reconstructed identically.
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
        self.cell_signature: Optional[str] = robot_cell.structural_signature() if robot_cell is not None else None
        self._actions: list[Action] = []

    def __iter__(self) -> Iterator[Action]:
        return iter(self._actions)

    def __len__(self) -> int:
        return len(self._actions)

    def iter_actions(self) -> Iterator[Action]:
        """Iterate over actions along the realized path."""
        return iter(self._actions)

    @property
    def actions(self) -> list[Action]:
        """Snapshot of the realized path. Returns a fresh list; mutating it does not affect the chain."""
        return list(self._actions)

    def iter_cell_states(self) -> Iterator["RobotCellState"]:
        """Yield a `RobotCellState` snapshot for every trajectory point and every state-change in the realized path.

        Useful for frame-by-frame playback: pair with an index slider and a
        visualisation component to scrub through the entire chain (not just one
        trajectory). Each yielded state is a fresh copy; mutating it does not
        affect the chain.

        Order: for a planned action, one snapshot per point in
        `action.trajectory.points`, with the pre-action configuration merged
        with the point's joint values (by name when joint_names are available,
        otherwise the point is used directly). For a state-change action, one
        snapshot — the explicit `post_state`.
        """
        running_state = self.start_state
        for action in self._actions:
            if action.is_trajectory:
                pre_state = running_state
                point_names = list(action.trajectory.points[0].joint_names) or list(getattr(action.trajectory, "joint_names", None) or [])
                base = pre_state.robot_configuration
                for point in action.trajectory.points:
                    snapshot = pre_state.copy()
                    if base is None or not point_names:
                        snapshot.robot_configuration = point
                    else:
                        snapshot.robot_configuration = base.merged(point)
                    yield snapshot
                running_state = action.post_state
            else:
                running_state = action.post_state
                yield running_state.copy()

    def action_by_name(self, name: str) -> Action:
        """Return the action with the given name.

        Raises
        ------
        KeyError
            If no action with that name exists.
        """
        for a in self._actions:
            if a.name == name:
                return a
        raise KeyError("No action named {!r} in chain {!r}".format(name, self.name))

    @property
    def end_state(self) -> "RobotCellState":
        if not self._actions:
            return self.start_state
        return cast("RobotCellState", self._actions[-1].post_state)

    @property
    def trajectories(self) -> list[JointTrajectory]:
        return [a.trajectory for a in self._actions if a.trajectory is not None]

    @property
    def duration(self) -> float:
        return sum(a.duration for a in self._actions)

    def append_trajectory(
        self,
        name: str,
        trajectory: JointTrajectory,
        description: str = "",
        tags: Optional[list[str]] = None,
        attributes: Optional[dict[str, Any]] = None,
    ) -> "ActionChain":
        """Append a planned (trajectory) action.

        The action's `start_state` is set to the current `end_state` and mirrored
        into `trajectory.start_state`. The `post_state` is derived from the
        start-state with the trajectory's last point applied. The trajectory's
        joint set must be a subset of the cell's configurable joints.

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
        # Action owns the start state; the trajectory mirrors it (stripped on serialize).
        trajectory.start_state = pre_state
        self._actions.append(
            Action(
                name=name,
                start_state=pre_state,
                trajectory=trajectory,
                post_state=post_state,
                description=description,
                tags=tags,
                attributes=attributes,
            )
        )
        return self

    def append_state_change(
        self,
        name: str,
        post_state: "RobotCellState",
        description: str = "",
        tags: Optional[list[str]] = None,
        attributes: Optional[dict[str, Any]] = None,
    ) -> "ActionChain":
        """Append a discrete state-change action (e.g. attach/detach tool, grasp/release).

        The action's `start_state` is set to the current `end_state`; `post_state`
        is the explicit state after the change.

        Returns `self` to allow chaining.

        Raises
        ------
        ValueError
            If `name` is already used.
        """
        self._assert_name_unused(name)
        self._actions.append(
            Action(
                name=name,
                start_state=self.end_state,
                post_state=post_state,
                description=description,
                tags=tags,
                attributes=attributes,
            )
        )
        return self

    def append_action(self, action: Action) -> "ActionChain":
        """Append a pre-built [Action][compas_fab.robots.Action].

        Dispatches to `append_trajectory()` or `append_state_change()` depending
        on whether the action carries a trajectory. Convenient when actions are
        constructed upstream (e.g. by Grasshopper builder components) and the
        caller just needs to thread them in order. The action's `attributes`
        (including `tags`) are preserved.

        Raises
        ------
        ValueError
            If `action.name` is already used, the trajectory references joints
            absent from the cell state, or a state-change action is missing its
            `post_state`.
        """
        if action.is_trajectory:
            return self.append_trajectory(
                action.name, action.trajectory, action.description, attributes=action.attributes
            )
        if action.post_state is None:
            raise ValueError("State-change action {!r} requires an explicit post_state".format(action.name))
        return self.append_state_change(
            action.name, action.post_state, action.description, attributes=action.attributes
        )

    def verify_cell(self, robot_cell: "RobotCell") -> None:
        """Verify the chain was assembled against `robot_cell`.

        Compares the cached cell signature against the given cell.

        Raises
        ------
        ValueError
            If the signatures differ. No-op when the chain was constructed
            without a bound cell.
        """
        if self.cell_signature is None:
            return
        actual = robot_cell.structural_signature()
        if actual != self.cell_signature:
            raise ValueError(
                "Chain {!r} was assembled against a different cell. "
                "Expected signature {}..., got {}...".format(self.name, self.cell_signature[:16], actual[:16])
            )

    def _assert_name_unused(self, name: str) -> None:
        if any(a.name == name for a in self._actions):
            raise ValueError("Action name {!r} already used in chain {!r}".format(name, self.name))

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

        actions_data = []
        for action in self._actions:
            d: dict[str, Any] = {"name": action.name, "description": action.description, "attributes": action.attributes}
            if action.trajectory is not None:
                td = action.trajectory.__data__
                td["start_state"] = None  # stripped — derivable by re-threading the chain
                d["trajectory"] = td
            else:
                d["post_state"] = action.post_state.__data__ if action.post_state is not None else None
            actions_data.append(d)
        return {
            "name": self.name,
            "description": self.description,
            "start_state": self.start_state.__data__,
            "cell_signature": self.cell_signature,
            "actions": actions_data,
        }

    @classmethod
    def __from_data__(cls, data: dict[str, Any]) -> "ActionChain":
        from compas_fab.robots import RobotCellState

        start_state = cast("RobotCellState", RobotCellState.__from_data__(data["start_state"]))
        chain = cls(name=data["name"], start_state=start_state, description=data.get("description", ""))
        chain.cell_signature = data.get("cell_signature")

        for ad in data["actions"]:
            attributes = ad.get("attributes")
            if "trajectory" in ad:
                traj = JointTrajectory.__from_data__(ad["trajectory"])
                chain.append_trajectory(ad["name"], traj, description=ad.get("description", ""), attributes=attributes)
            else:
                post_state = cast("RobotCellState", RobotCellState.__from_data__(ad["post_state"]))
                chain.append_state_change(
                    ad["name"], post_state, description=ad.get("description", ""), attributes=attributes
                )
        return chain
