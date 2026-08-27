import math

from compas.geometry import Frame
from compas.geometry import Quaternion
from compas.geometry import Transformation
from compas_robots import Configuration

from compas_fab.backends.exceptions import BackendTargetNotSupportedError
from compas_fab.backends.exceptions import CollisionCheckError
from compas_fab.backends.exceptions import InverseKinematicsError
from compas_fab.backends.exceptions import PlanningGroupNotExistsError
from compas_fab.backends.interfaces import InverseKinematics
from compas_fab.robots import FrameTarget
from compas_fab.robots import PointAxisTarget

from ..collision import collision_constraints
from ..collision import compile_collision_scene
from ..conversions import frame_to_wxyz_xyz
from ..problem_cache import collision_scene_key
from ..problem_cache import inactive_configuration_key


def _quaternion_distance(frame_a, frame_b):
    quaternion_a = Quaternion.from_frame(frame_a)
    quaternion_b = Quaternion.from_frame(frame_b)
    dot = abs(quaternion_a.w * quaternion_b.w + quaternion_a.x * quaternion_b.x + quaternion_a.y * quaternion_b.y + quaternion_a.z * quaternion_b.z)
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def _axis_distance(axis_a, axis_b):
    dot = axis_a.dot(axis_b) / (axis_a.length * axis_b.length)
    return math.acos(min(1.0, max(-1.0, dot)))


def _assert_start_configuration(robot_cell, configuration):
    expected_names = robot_cell.robot_model.get_configurable_joint_names()
    if configuration is None:
        raise ValueError("RobotCellState.robot_configuration is required for numerical IK.")
    missing = [name for name in expected_names if name not in configuration.keys()]
    if missing:
        raise ValueError("The start configuration is missing joints: {}".format(", ".join(missing)))
    for name in expected_names:
        value = float(configuration[name])
        if not math.isfinite(value):
            raise ValueError("The start configuration contains a non-finite value for joint '{}'.".format(name))
        joint = robot_cell.robot_model.get_joint_by_name(name)
        if joint.limit and not joint.limit.lower <= value <= joint.limit.upper:
            raise ValueError("The start configuration violates the limits of joint '{}'.".format(name))


def _target_tolerances(target, options, default_position, default_orientation):
    position_tolerance = target.tolerance_position
    if position_tolerance is None:
        position_tolerance = options.get("position_tolerance", default_position)
    orientation_tolerance = target.tolerance_orientation
    if orientation_tolerance is None:
        orientation_tolerance = options.get("orientation_tolerance", default_orientation)
    return position_tolerance, orientation_tolerance


class PyRokiInverseKinematics(InverseKinematics):
    """Single-seed differentiable IK for FAB frame and point-axis targets.

    Collision constraints are enabled by default, consistent with FAB's other
    numerical IK backends. Pass ``check_collision=False`` for a
    kinematics-only query. ``collision_margin`` specifies additional clearance
    in meters and defaults to zero.
    """

    DEFAULT_POSITION_TOLERANCE = 1e-4
    DEFAULT_ORIENTATION_TOLERANCE = 1e-3

    def iter_inverse_kinematics(self, target, robot_cell_state=None, group=None, options=None):
        options = dict(options or {})
        robot_cell = self.client.robot_cell
        if robot_cell is None:
            raise ValueError("Set a RobotCell before requesting inverse kinematics.")

        group = group or robot_cell.main_group_name
        if group not in robot_cell.group_names:
            raise PlanningGroupNotExistsError("Planning group '{}' is not supported by the PyRoKI planner.".format(group))
        if not isinstance(target, (FrameTarget, PointAxisTarget)):
            raise BackendTargetNotSupportedError("The PyRoKI backend currently supports FrameTarget and PointAxisTarget.")
        options["check_collision"] = options.get("check_collision", True)

        target = target.normalized_to_meters()
        robot_cell.assert_cell_state_match(robot_cell_state)
        robot_cell_state.assert_target_mode_match(target.target_mode, group)
        _assert_start_configuration(robot_cell, robot_cell_state.robot_configuration)
        self.set_robot_cell_state(robot_cell_state)
        scene_key = collision_scene_key(robot_cell_state)
        collision_scene = self._collision_scene(robot_cell, robot_cell_state, scene_key) if options["check_collision"] else None
        if collision_scene is not None:
            static_collision_pairs = collision_scene.static_colliding_pairs(float(options.get("collision_margin", 0.0)))
            if static_collision_pairs:
                raise CollisionCheckError(
                    "The FAB scene contains a collision that robot joints cannot resolve: {}".format(
                        ", ".join("'{}' with '{}'".format(name_a, name_b) for name_a, name_b in static_collision_pairs)
                    ),
                    static_collision_pairs,
                )

        if isinstance(target, FrameTarget):
            solution_values = self._solve_frame_target(target, robot_cell_state, group, options, collision_scene, scene_key)
        else:
            solution_values = self._solve_point_axis_target(target, robot_cell_state, group, options, collision_scene, scene_key)

        if collision_scene is not None:
            self._assert_collision_free(solution_values, collision_scene, options)

        yield self._result_configuration(solution_values, group, options)

    def _solve_frame_target(self, target, robot_cell_state, group, options, collision_scene, scene_key):
        import jax.numpy as jnp
        import jaxlie
        import pyroki

        robot_cell = self.client.robot_cell
        target_pcf = robot_cell.target_frames_to_pcf(robot_cell_state, target.target_frame, target.target_mode, group)
        target_base = robot_cell_state.robot_base_frame.to_local_coordinates(target_pcf)
        model, joint_var, joint_mask = self._solver_inputs(group)
        target_link_index = model.link_names.index(robot_cell.get_end_effector_link_name(group))
        target_pose = jaxlie.SE3(jnp.asarray(frame_to_wxyz_xyz(target_base)))
        problem_key = self._problem_cache_key("frame", group, robot_cell_state, scene_key, options)
        problem_template = self.client._problem_cache.get_ik_problem(problem_key)
        if problem_template is None:
            costs = [
                pyroki.costs.pose_cost_analytic_jac(
                    model.robot,
                    joint_var,
                    target_pose,
                    jnp.asarray(target_link_index, dtype=jnp.int32),
                    pos_weight=float(options.get("position_weight", 50.0)),
                    ori_weight=float(options.get("orientation_weight", 10.0)),
                    joint_mask=joint_mask,
                ),
                pyroki.costs.limit_constraint(model.robot, joint_var),
            ]
            costs.extend(self._collision_constraints(collision_scene, model, joint_var, joint_mask, robot_cell_state, options))
            problem_template = self.client._problem_cache.put_ik_problem(problem_key, self._analyze(costs, joint_var))
        problem = problem_template.with_cost_arguments([("_pose_cost_analytical_jac", 2, target_pose)])
        solution = self._solve(problem, joint_var, robot_cell_state.robot_configuration, group, options)

        configuration = self._full_configuration(solution)
        actual_base = robot_cell.robot_model.forward_kinematics(configuration, robot_cell.get_end_effector_link_name(group))
        position_tolerance, orientation_tolerance = _target_tolerances(
            target,
            options,
            self.DEFAULT_POSITION_TOLERANCE,
            self.DEFAULT_ORIENTATION_TOLERANCE,
        )
        position_error = actual_base.point.distance_to_point(target_base.point)
        orientation_error = _quaternion_distance(actual_base, target_base)
        if position_error > position_tolerance or orientation_error > orientation_tolerance:
            raise InverseKinematicsError(
                "PyRoKI did not reach the FrameTarget within tolerance (position error {:.3g} m, orientation error {:.3g} rad).".format(position_error, orientation_error)
            )
        return solution

    def _solve_point_axis_target(self, target, robot_cell_state, group, options, collision_scene, scene_key):
        import jax.numpy as jnp
        import jaxlie
        import jaxls
        import pyroki

        robot_cell = self.client.robot_cell
        model, joint_var, joint_mask = self._solver_inputs(group)
        target_link_index = model.link_names.index(robot_cell.get_end_effector_link_name(group))

        t_base_world = Transformation.from_frame(robot_cell_state.robot_base_frame).inverted()
        target_point_base = target.target_point.transformed(t_base_world)
        target_axis_base = target.target_z_axis.transformed(t_base_world).unitized()

        reference_offset = robot_cell.pcf_to_target_frames(robot_cell_state, Frame.worldXY(), target.target_mode, group)
        reference_offset_pose = jaxlie.SE3(jnp.asarray(frame_to_wxyz_xyz(reference_offset)))
        start_values = jnp.asarray(model.configuration_values(robot_cell_state.robot_configuration), dtype=float)

        def point_axis_residual(
            variable_values,
            robot,
            variable,
            link_index,
            offset_pose,
            target_position,
            target_axis,
            initial_values,
            active_mask,
            position_weight,
            orientation_weight,
        ):
            configuration = jnp.where(active_mask > 0.0, variable_values[variable], initial_values)
            link_pose = jaxlie.SE3(robot.forward_kinematics(configuration)[link_index])
            reference_pose = link_pose @ offset_pose
            position_residual = (reference_pose.translation() - target_position) * position_weight
            z_axis = reference_pose.rotation().as_matrix()[:, 2]
            orientation_residual = (z_axis - target_axis) * orientation_weight
            return jnp.concatenate([position_residual, orientation_residual])

        target_position = jnp.asarray(list(target_point_base))
        target_axis = jnp.asarray(list(target_axis_base))
        problem_key = self._problem_cache_key("point_axis", group, robot_cell_state, scene_key, options, target.target_mode)
        problem_template = self.client._problem_cache.get_ik_problem(problem_key)
        if problem_template is None:
            point_axis_cost = jaxls.Cost.factory(point_axis_residual)
            costs = [
                point_axis_cost(
                    model.robot,
                    joint_var,
                    jnp.asarray(target_link_index, dtype=jnp.int32),
                    reference_offset_pose,
                    target_position,
                    target_axis,
                    start_values,
                    joint_mask,
                    float(options.get("position_weight", 50.0)),
                    float(options.get("orientation_weight", 10.0)),
                ),
                pyroki.costs.limit_constraint(model.robot, joint_var),
            ]
            costs.extend(self._collision_constraints(collision_scene, model, joint_var, joint_mask, robot_cell_state, options))
            problem_template = self.client._problem_cache.put_ik_problem(problem_key, self._analyze(costs, joint_var))
        problem = problem_template.with_cost_arguments(
            [
                ("point_axis_residual", 4, target_position),
                ("point_axis_residual", 5, target_axis),
                ("point_axis_residual", 6, start_values),
            ]
        )
        solution = self._solve(problem, joint_var, robot_cell_state.robot_configuration, group, options)

        configuration = self._full_configuration(solution)
        actual_pcf_base = robot_cell.robot_model.forward_kinematics(configuration, robot_cell.get_end_effector_link_name(group))
        actual_reference_base = Frame.from_transformation(Transformation.from_frame(actual_pcf_base) * Transformation.from_frame(reference_offset))
        position_tolerance, orientation_tolerance = _target_tolerances(
            target,
            options,
            self.DEFAULT_POSITION_TOLERANCE,
            self.DEFAULT_ORIENTATION_TOLERANCE,
        )
        position_error = actual_reference_base.point.distance_to_point(target_point_base)
        orientation_error = _axis_distance(actual_reference_base.zaxis, target_axis_base)
        if position_error > position_tolerance or orientation_error > orientation_tolerance:
            raise InverseKinematicsError(
                "PyRoKI did not reach the PointAxisTarget within tolerance (position error {:.3g} m, axis error {:.3g} rad).".format(position_error, orientation_error)
            )
        return solution

    def _collision_constraints(self, scene, model, joint_var, joint_mask, robot_cell_state, options):
        if scene is None or not scene.has_constraints:
            return []
        import jax.numpy as jnp

        initial_values = jnp.asarray(model.configuration_values(robot_cell_state.robot_configuration), dtype=float)
        return collision_constraints(
            scene,
            model.robot,
            joint_var,
            initial_values,
            joint_mask,
            margin=float(options.get("collision_margin", 0.0)),
            weight=float(options.get("collision_weight", 10.0)),
        )

    def _assert_collision_free(self, solution, scene, options):
        model = self.client.pyroki_model
        collision_pairs = scene.colliding_pairs(
            model.robot,
            solution,
            margin=float(options.get("collision_margin", 0.0)),
        )
        if collision_pairs:
            message = "PyRoKI IK returned a configuration in capsule-approximated collision: {}".format(
                ", ".join("'{}' with '{}'".format(name_a, name_b) for name_a, name_b in collision_pairs)
            )
            raise CollisionCheckError(message, collision_pairs)

    def _solver_inputs(self, group):
        import jax.numpy as jnp

        model = self.client.pyroki_model
        joint_var = model.robot.joint_var_cls(0)
        active_joint_names = set(self.client.robot_cell.get_configurable_joint_names(group))
        joint_mask = jnp.asarray([1.0 if name in active_joint_names else 0.0 for name in model.joint_names])
        return model, joint_var, joint_mask

    def _collision_scene(self, robot_cell, robot_cell_state, scene_key):
        scene = self.client._problem_cache.get_collision_scene(scene_key)
        if scene is None:
            scene = compile_collision_scene(robot_cell, robot_cell_state)
            self.client._problem_cache.put_collision_scene(scene_key, scene)
        return scene

    def _problem_cache_key(self, target_kind, group, robot_cell_state, scene_key, options, target_mode=None):
        model = self.client.pyroki_model
        active_joint_names = self.client.robot_cell.get_configurable_joint_names(group)
        return (
            target_kind,
            group,
            str(target_mode),
            scene_key,
            inactive_configuration_key(model, robot_cell_state.robot_configuration, active_joint_names),
            bool(options.get("check_collision", True)),
            float(options.get("position_weight", 50.0)),
            float(options.get("orientation_weight", 10.0)),
            float(options.get("collision_margin", 0.0)),
            float(options.get("collision_weight", 10.0)),
        )

    @staticmethod
    def _analyze(costs, joint_var):
        import jaxls

        return jaxls.LeastSquaresProblem(costs=costs, variables=[joint_var]).analyze(schur_elimination="off")

    def _solve(self, problem, joint_var, start_configuration, group, options):
        import jax.numpy as jnp
        import jaxls

        model = self.client.pyroki_model
        start_values = jnp.asarray(model.configuration_values(start_configuration), dtype=float)
        initial_values = jaxls.VarValues.make([joint_var.with_value(start_values)])
        try:
            result = problem.solve(
                initial_vals=initial_values,
                verbose=bool(options.get("verbose", False)),
                linear_solver="dense_cholesky",
                trust_region=jaxls.TrustRegionConfig(lambda_initial=float(options.get("lambda_initial", 1.0))),
                termination=jaxls.TerminationConfig(max_iterations=int(options.get("max_iterations", 100))),
            )
        except (FloatingPointError, ValueError) as error:
            raise InverseKinematicsError("PyRoKI failed to solve the target: {}".format(error)) from error

        solution = [float(value) for value in result[joint_var]]
        active_joint_names = set(self.client.robot_cell.get_configurable_joint_names(group))
        for index, name in enumerate(model.joint_names):
            if name not in active_joint_names:
                solution[index] = float(start_configuration[name])
            if not math.isfinite(solution[index]):
                raise InverseKinematicsError("PyRoKI returned a non-finite value for joint '{}'.".format(name))
            joint = self.client.robot_model.get_joint_by_name(name)
            if joint.limit and not joint.limit.lower - 1e-9 <= solution[index] <= joint.limit.upper + 1e-9:
                raise InverseKinematicsError("PyRoKI returned a value outside the limits of joint '{}'.".format(name))
        return solution

    def _full_configuration(self, solution):
        model = self.client.pyroki_model
        joint_names = list(model.joint_names)
        joint_types = self.client.robot_model.get_joint_types_by_names(joint_names)
        return Configuration(solution, joint_types, joint_names)

    def _result_configuration(self, solution, group, options):
        full_configuration = self._full_configuration(solution)
        if options.get("return_full_configuration", False):
            return full_configuration
        robot_cell = self.client.robot_cell
        group_joint_names = robot_cell.get_configurable_joint_names(group)
        return Configuration(
            [full_configuration[name] for name in group_joint_names],
            robot_cell.get_configurable_joint_types(group),
            group_joint_names,
        )
