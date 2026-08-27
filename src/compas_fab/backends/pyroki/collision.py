from dataclasses import dataclass
from dataclasses import field

from compas.geometry import Frame
from compas.geometry import Transformation


@dataclass(frozen=True)
class _GeometryRecord:
    geometry: object
    link_index: int
    owner_name: str
    owner_kind: str
    entity_kind: str
    touch_links: frozenset = field(default_factory=frozenset)
    touch_bodies: frozenset = field(default_factory=frozenset)
    attached_to_body: str = None


@dataclass(frozen=True)
class PyRokiCollisionScene:
    """Differentiable capsule approximation of a FAB collision scene."""

    robot_collision: object = None
    world_collision: object = None
    self_pairs: tuple = ()
    world_pairs: tuple = ()
    world_robot_indices: tuple = ()
    world_geometry_indices: tuple = ()
    static_pairs: tuple = ()
    static_distances: tuple = ()

    @property
    def has_constraints(self):
        return bool(self.self_pairs or self.world_pairs)

    def static_colliding_pairs(self, margin=0.0):
        """Return joint-independent scene collisions closer than ``margin``."""
        return [pair for pair, distance in zip(self.static_pairs, self.static_distances) if distance < margin]

    def colliding_pairs(self, robot, configuration_values, margin=0.0):
        """Return approximated collision pairs closer than ``margin``."""
        import jax.numpy as jnp

        collisions = self.static_colliding_pairs(margin)
        if self.robot_collision is None:
            return collisions
        if self.self_pairs:
            distances = self.robot_collision.compute_self_collision_distance(robot, jnp.asarray(configuration_values))
            collisions.extend(pair for pair, distance in zip(self.self_pairs, distances) if float(distance) < margin)

        if self.world_pairs:
            distances = self.robot_collision.compute_world_collision_distance(robot, jnp.asarray(configuration_values), self.world_collision)
            active_distances = distances[jnp.asarray(self.world_robot_indices), jnp.asarray(self.world_geometry_indices)]
            collisions.extend(pair for pair, distance in zip(self.world_pairs, active_distances) if float(distance) < margin)
        return collisions


def _shape_meshes_or_raise(model, model_name):
    missing = []
    for link in model.links:
        for collision in link.collision:
            if not collision.geometry.shape.meshes:
                missing.append(link.name)
    if missing:
        raise ValueError(
            "Collision geometry for {} is not loaded on links: {}. Load the cell with geometry or pass options={{'check_collision': False}}.".format(
                model_name,
                ", ".join(sorted(set(missing))),
            )
        )


def _mesh_vertices_and_faces(mesh, transformation=None):
    if transformation is not None:
        mesh = mesh.transformed(transformation)
    vertices, faces = mesh.to_vertices_and_faces(triangulated=True)
    if not vertices or not faces:
        raise ValueError("A configured collision mesh is empty.")
    return vertices, faces


def _capsule_from_mesh(mesh, transformation=None):
    import jax.numpy as jnp
    import trimesh
    from pyroki.collision import Capsule

    vertices, faces = _mesh_vertices_and_faces(mesh, transformation)
    trimesh_mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    capsule = Capsule.from_trimesh(trimesh_mesh)

    # PyRoKI's fitted cylinder height includes both end caps. Convert that
    # extent to the cylindrical segment used by its Capsule representation.
    height = jnp.maximum(0.0, capsule.height - 2.0 * capsule.radius)
    return Capsule.from_radius_height(
        radius=capsule.radius,
        height=height,
        position=capsule.pose.translation(),
        wxyz=capsule.pose.rotation().wxyz,
    )


def _box_from_mesh(mesh, transformation=None):
    import jax.numpy as jnp
    from pyroki.collision import Box

    vertices, _ = _mesh_vertices_and_faces(mesh, transformation)
    vertices = jnp.asarray(vertices)
    lower = jnp.min(vertices, axis=0)
    upper = jnp.max(vertices, axis=0)
    extent = jnp.maximum(upper - lower, 1e-6)
    return Box.from_extent(extent=extent, position=(lower + upper) / 2.0)


def _stack_geometries(records):
    import jax
    import jax.numpy as jnp

    geometries = [record.geometry for record in records]
    return jax.tree.map(lambda *values: jnp.stack(values), *geometries)


def _model_meshes_in_base(model, configuration=None):
    configuration = configuration or model.zero_configuration()
    for link in model.links:
        link_frame = model.forward_kinematics(configuration, link.name)
        transformation = Transformation.from_frame(link_frame)
        for mesh in model.get_link_collision_meshes(link):
            yield mesh, transformation


def _robot_records(robot_cell):
    _shape_meshes_or_raise(robot_cell.robot_model, "robot '{}'".format(robot_cell.robot_model.name))
    records = []
    link_indices = {name: index for index, name in enumerate(link.name for link in robot_cell.robot_model.links)}
    for link in robot_cell.robot_model.links:
        for mesh in robot_cell.robot_model.get_link_collision_meshes(link):
            records.append(
                _GeometryRecord(
                    geometry=_capsule_from_mesh(mesh),
                    link_index=link_indices[link.name],
                    owner_name=link.name,
                    owner_kind="link",
                    entity_kind="link",
                )
            )
    return records, link_indices


def _tool_records(robot_cell, state, link_indices, t_base_world):
    robot_records = []
    world_records = []
    tool_locations = {}

    for tool_id, tool_model in robot_cell.tool_models.items():
        tool_state = state.tool_states[tool_id]
        if tool_state.is_hidden:
            continue
        _shape_meshes_or_raise(tool_model, "tool '{}'".format(tool_id))
        configuration = tool_state.configuration or tool_model.zero_configuration()
        touch_links = frozenset(tool_state.touch_links)

        if tool_state.attached_to_group:
            link_name = robot_cell.get_end_effector_link_name(tool_state.attached_to_group)
            t_link_tool = Transformation.from_frame(tool_state.attachment_frame or Frame.worldXY())
            tool_locations[tool_id] = ("robot", link_name, t_link_tool)
            for mesh, t_tool_link in _model_meshes_in_base(tool_model, configuration):
                robot_records.append(
                    _GeometryRecord(
                        geometry=_capsule_from_mesh(mesh, t_link_tool * t_tool_link),
                        link_index=link_indices[link_name],
                        owner_name=tool_id,
                        owner_kind="body",
                        entity_kind="tool",
                        touch_links=touch_links,
                    )
                )
        else:
            t_base_tool = t_base_world * Transformation.from_frame(tool_state.frame)
            tool_locations[tool_id] = ("world", None, t_base_tool)
            for mesh, t_tool_link in _model_meshes_in_base(tool_model, configuration):
                world_records.append(
                    _GeometryRecord(
                        geometry=_box_from_mesh(mesh, t_base_tool * t_tool_link),
                        link_index=-1,
                        owner_name=tool_id,
                        owner_kind="body",
                        entity_kind="tool",
                        touch_links=touch_links,
                    )
                )
    return robot_records, world_records, tool_locations


def _rigid_body_records(robot_cell, state, link_indices, t_base_world, tool_locations):
    robot_records = []
    world_records = []
    for body_id, body_model in robot_cell.rigid_body_models.items():
        body_state = state.rigid_body_states[body_id]
        if body_state.is_hidden or not body_model.collision_meshes:
            continue

        touch_links = frozenset(body_state.touch_links)
        touch_bodies = frozenset(body_state.touch_bodies)
        if body_state.attached_to_link:
            link_name = body_state.attached_to_link
            t_link_body = Transformation.from_frame(body_state.attachment_frame or Frame.worldXY())
            destination = robot_records
            link_index = link_indices[link_name]
            transformation = t_link_body
        elif body_state.attached_to_tool:
            if body_state.attached_to_tool not in tool_locations:
                raise ValueError("Rigid body '{}' is attached to hidden or unavailable tool '{}'.".format(body_id, body_state.attached_to_tool))
            tool_location, link_name, t_parent_tool = tool_locations[body_state.attached_to_tool]
            tool_model = robot_cell.tool_models[body_state.attached_to_tool]
            t_tool_body = Transformation.from_frame(tool_model.frame) * Transformation.from_frame(body_state.attachment_frame or Frame.worldXY())
            transformation = t_parent_tool * t_tool_body
            if tool_location == "robot":
                destination = robot_records
                link_index = link_indices[link_name]
            else:
                destination = world_records
                link_index = -1
        else:
            destination = world_records
            link_index = -1
            transformation = t_base_world * Transformation.from_frame(body_state.frame)

        for mesh in body_model.collision_meshes_in_meters:
            destination.append(
                _GeometryRecord(
                    geometry=(_capsule_from_mesh(mesh, transformation) if destination is robot_records else _box_from_mesh(mesh, transformation)),
                    link_index=link_index,
                    owner_name=body_id,
                    owner_kind="body",
                    entity_kind="rigid_body",
                    touch_links=touch_links,
                    touch_bodies=touch_bodies,
                    attached_to_body=body_state.attached_to_tool,
                )
            )
    return robot_records, world_records


def _self_pair_is_active(record_a, record_b, disabled_collisions):
    if record_a.owner_name == record_b.owner_name and record_a.owner_kind == record_b.owner_kind:
        return False
    if record_a.owner_kind == record_b.owner_kind == "link":
        return frozenset((record_a.owner_name, record_b.owner_name)) not in disabled_collisions
    if record_a.owner_kind == "link":
        return record_a.owner_name not in record_b.touch_links
    if record_b.owner_kind == "link":
        return record_b.owner_name not in record_a.touch_links
    if record_a.attached_to_body == record_b.owner_name or record_b.attached_to_body == record_a.owner_name:
        return False
    return record_b.owner_name not in record_a.touch_bodies and record_a.owner_name not in record_b.touch_bodies


def _world_pair_is_active(robot_record, world_record):
    if robot_record.owner_kind == "link":
        return robot_record.owner_name not in world_record.touch_links
    return world_record.owner_name not in robot_record.touch_bodies and robot_record.owner_name not in world_record.touch_bodies


def _static_pair_is_active(record_a, record_b):
    if record_a.owner_name == record_b.owner_name:
        return False
    if record_a.attached_to_body == record_b.owner_name or record_b.attached_to_body == record_a.owner_name:
        return False
    if record_b.owner_name in record_a.touch_bodies or record_a.owner_name in record_b.touch_bodies:
        return False
    kinds = {record_a.entity_kind, record_b.entity_kind}
    if kinds == {"tool", "rigid_body"}:
        return True
    return kinds == {"rigid_body"} and bool(record_a.attached_to_body or record_b.attached_to_body)


def _box_distance(box_a, box_b):
    import jax.numpy as jnp

    delta = jnp.abs(box_a.pose.translation() - box_b.pose.translation()) - (box_a.half_extent + box_b.half_extent)
    outside_distance = jnp.linalg.norm(jnp.maximum(delta, 0.0))
    inside_distance = jnp.minimum(jnp.max(delta), 0.0)
    return float(outside_distance + inside_distance)


def compile_collision_scene(robot_cell, state):
    """Compile FAB collision geometry and state into PyRoKI primitives.

    Moving meshes are approximated independently by fitted capsules and static
    world meshes by axis-aligned boxes. Hidden objects and FAB's
    allowed-collision semantics are retained.
    """
    import jax.numpy as jnp
    from pyroki.collision import RobotCollision

    robot_cell.assert_cell_state_match(state)
    t_base_world = Transformation.from_frame(state.robot_base_frame).inverted()
    robot_records, link_indices = _robot_records(robot_cell)
    tool_robot, tool_world, tool_locations = _tool_records(robot_cell, state, link_indices, t_base_world)
    body_robot, body_world = _rigid_body_records(robot_cell, state, link_indices, t_base_world, tool_locations)
    robot_records.extend(tool_robot)
    robot_records.extend(body_robot)
    world_records = tool_world + body_world

    if not robot_records:
        return PyRokiCollisionScene()

    disabled_collisions = robot_cell.robot_semantics.unordered_disabled_collisions
    active_i = []
    active_j = []
    self_pairs = []
    for index_a, record_a in enumerate(robot_records):
        for index_b in range(index_a + 1, len(robot_records)):
            record_b = robot_records[index_b]
            if _self_pair_is_active(record_a, record_b, disabled_collisions):
                active_i.append(index_a)
                active_j.append(index_b)
                self_pairs.append((record_a.owner_name, record_b.owner_name))

    robot_collision = RobotCollision(
        num_links=len(robot_cell.robot_model.links),
        link_names=tuple(link.name for link in robot_cell.robot_model.links),
        coll=_stack_geometries(robot_records),
        active_idx_i=tuple(active_i),
        active_idx_j=tuple(active_j),
        _geom_to_link_idx=jnp.asarray([record.link_index for record in robot_records], dtype=jnp.int32),
    )

    world_collision = _stack_geometries(world_records) if world_records else None
    world_robot_indices = []
    world_geometry_indices = []
    world_pairs = []
    for robot_index, robot_record in enumerate(robot_records):
        for world_index, world_record in enumerate(world_records):
            if _world_pair_is_active(robot_record, world_record):
                world_robot_indices.append(robot_index)
                world_geometry_indices.append(world_index)
                world_pairs.append((robot_record.owner_name, world_record.owner_name))

    static_pairs = []
    static_distances = []
    for index_a, record_a in enumerate(world_records):
        for index_b in range(index_a + 1, len(world_records)):
            record_b = world_records[index_b]
            if _static_pair_is_active(record_a, record_b):
                static_pairs.append((record_a.owner_name, record_b.owner_name))
                static_distances.append(_box_distance(record_a.geometry, record_b.geometry))

    return PyRokiCollisionScene(
        robot_collision=robot_collision,
        world_collision=world_collision,
        self_pairs=tuple(self_pairs),
        world_pairs=tuple(world_pairs),
        world_robot_indices=tuple(world_robot_indices),
        world_geometry_indices=tuple(world_geometry_indices),
        static_pairs=tuple(static_pairs),
        static_distances=tuple(static_distances),
    )


def collision_constraints(scene, robot, joint_var, initial_values, active_mask, margin, weight):
    """Create differentiable hard constraints for active FAB collision pairs."""
    import jax.numpy as jnp
    import jaxls

    def masked_configuration(variable_values, variable, initial, mask):
        return jnp.where(mask > 0.0, variable_values[variable], initial)

    def self_collision_residual(variable_values, robot, robot_collision, variable, initial, mask, margin, weight):
        configuration = masked_configuration(variable_values, variable, initial, mask)
        distances = robot_collision.compute_self_collision_distance(robot, configuration)
        return (margin - distances) * weight

    def world_collision_residual(
        variable_values,
        robot,
        robot_collision,
        variable,
        world_collision,
        robot_indices,
        world_indices,
        initial,
        mask,
        margin,
        weight,
    ):
        configuration = masked_configuration(variable_values, variable, initial, mask)
        distances = robot_collision.compute_world_collision_distance(robot, configuration, world_collision)
        active_distances = distances[robot_indices, world_indices]
        return (margin - active_distances) * weight

    constraints = []
    constraint_factory = jaxls.Cost.factory(kind="constraint_leq_zero")
    if scene.self_pairs:
        constraints.append(
            constraint_factory(self_collision_residual)(
                robot,
                scene.robot_collision,
                joint_var,
                initial_values,
                active_mask,
                float(margin),
                float(weight),
            )
        )
    if scene.world_pairs:
        constraints.append(
            constraint_factory(world_collision_residual)(
                robot,
                scene.robot_collision,
                joint_var,
                scene.world_collision,
                jnp.asarray(scene.world_robot_indices, dtype=jnp.int32),
                jnp.asarray(scene.world_geometry_indices, dtype=jnp.int32),
                initial_values,
                active_mask,
                float(margin),
                float(weight),
            )
        )
    return constraints
