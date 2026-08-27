from compas_fab.backends.exceptions import CollisionCheckError
from compas_fab.backends.interfaces import CheckCollision

from ..collision import compile_collision_scene


class PyRokiCheckCollision(CheckCollision):
    """Check a FAB cell state using the PyRoKI capsule approximation."""

    def check_collision(self, robot_cell_state, options=None):
        options = dict(options or {})
        robot_cell = self.client.robot_cell
        if robot_cell is None:
            raise ValueError("Set a RobotCell before checking collisions.")
        robot_cell.assert_cell_state_match(robot_cell_state)
        if robot_cell_state.robot_configuration is None:
            raise ValueError("RobotCellState.robot_configuration is required for collision checking.")

        self.set_robot_cell_state(robot_cell_state)
        scene = compile_collision_scene(robot_cell, robot_cell_state)
        values = self.client.pyroki_model.configuration_values(robot_cell_state.robot_configuration)
        collision_pairs = scene.colliding_pairs(
            self.client.pyroki_model.robot,
            values,
            margin=float(options.get("collision_margin", 0.0)),
        )
        if collision_pairs:
            if not options.get("full_report", False):
                collision_pairs = collision_pairs[:1]
            message = "PyRoKI capsule collision detected: {}".format(", ".join("'{}' with '{}'".format(name_a, name_b) for name_a, name_b in collision_pairs))
            raise CollisionCheckError(message, collision_pairs)
