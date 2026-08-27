from collections import OrderedDict
from dataclasses import dataclass
from dataclasses import replace

import compas


def collision_scene_key(robot_cell_state):
    """Return the collision-relevant portion of a robot cell state.

    Active robot joint values do not change the compiled collision geometry:
    PyRoKI applies those values through forward kinematics at solve time. Tool
    and rigid-body state, including their attachment and hidden state, does.
    """
    return compas.json_dumps(
        {
            "robot_base_frame": robot_cell_state.robot_base_frame,
            "tool_states": robot_cell_state.tool_states,
            "rigid_body_states": robot_cell_state.rigid_body_states,
        },
        compact=True,
        minimal=True,
    )


def inactive_configuration_key(model, configuration, active_joint_names):
    """Return values that are fixed while solving a planning group."""
    active_joint_names = set(active_joint_names)
    return tuple((name, float(configuration[name])) for name in model.joint_names if name not in active_joint_names)


@dataclass(frozen=True)
class AnalyzedProblemTemplate:
    """Analyzed JAX least-squares problem with replaceable runtime arguments."""

    problem: object

    def with_cost_arguments(self, updates):
        """Replace unbatched positional cost arguments without re-analysis."""
        import jax
        import jax.numpy as jnp

        updates_by_name = {}
        for cost_name, argument_index, value in updates:
            updates_by_name.setdefault(cost_name, {})[argument_index] = value

        found = set()
        stacked_costs = []
        for cost in self.problem._stacked_costs:
            replacements = updates_by_name.get(cost.name)
            if replacements:
                positional, keyword = cost.args
                positional = list(positional)
                for argument_index, value in replacements.items():
                    positional[argument_index] = jax.tree.map(lambda leaf: jnp.asarray(leaf)[None], value)
                cost = replace(cost, args=(tuple(positional), keyword))
                found.add(cost.name)
            stacked_costs.append(cost)

        missing = set(updates_by_name) - found
        if missing:
            raise RuntimeError("Cached PyRoKI problem is missing costs: {}".format(", ".join(sorted(missing))))
        return replace(self.problem, _stacked_costs=tuple(stacked_costs))


class PyRokiProblemCache:
    """Small LRU caches for collision geometry and analyzed IK problems."""

    def __init__(self, maxsize=4):
        self.maxsize = maxsize
        self.collision_scenes = OrderedDict()
        self.ik_problems = OrderedDict()

    @staticmethod
    def _get(cache, key):
        value = cache.get(key)
        if value is not None:
            cache.move_to_end(key)
        return value

    def get_collision_scene(self, key):
        return self._get(self.collision_scenes, key)

    def put_collision_scene(self, key, scene):
        self._put(self.collision_scenes, key, scene)

    def get_ik_problem(self, key):
        return self._get(self.ik_problems, key)

    def put_ik_problem(self, key, problem):
        template = AnalyzedProblemTemplate(problem)
        self._put(self.ik_problems, key, template)
        return template

    def _put(self, cache, key, value):
        cache[key] = value
        cache.move_to_end(key)
        while len(cache) > self.maxsize:
            cache.popitem(last=False)

    def clear(self):
        self.collision_scenes.clear()
        self.ik_problems.clear()
