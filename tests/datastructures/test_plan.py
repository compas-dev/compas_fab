from itertools import count

import compas
import pytest

from compas.geometry import Frame
from compas_fab.datastructures import Action
from compas_fab.datastructures import IntegerIdGenerator
from compas_fab.datastructures import PartialOrder
from compas_fab.datastructures import DependencyIdException


def test_action_data():
    name = "action"
    parameters = {'param1': 1, 'param2': Frame.worldXY()}
    action = Action(name, parameters)
    other_action = Action.from_data(action.data)
    assert action.name == other_action.name
    assert action.parameters == other_action.parameters

    action_json = compas.json_dumps(action.data)
    other_action_data = compas.json_loads(action_json)
    other_action = Action.from_data(other_action_data)
    assert other_action.parameters['param2'] == Frame.worldXY()


def test_integer_id_generator():
    generator = IntegerIdGenerator()
    assert next(generator) == 1
    assert next(generator) == 2
    generator_json = compas.json_dumps(generator.data)
    generator_data = compas.json_loads(generator_json)
    generator_reincarnate = IntegerIdGenerator.from_data(generator_data)
    assert next(generator_reincarnate) == 3


def test_plan():
    plan = PartialOrder()
    assert len(plan.actions) == 0


def test_plan_generator_compatibility():
    action = Action('action', {'param': 3})
    plan = PartialOrder()
    action_id = plan.plan_action(action, [])
    assert action_id == 1
    assert next(plan._id_generator) == 2


def test_plan_with_custom_id_generator():
    class CustomIdGenerator(object):
        def __init__(self):
            self._generator = count(ord('a'))

        def __next__(self):
            return chr(next(self._generator))

        # alias for ironpython
        next = __next__

    plan = PartialOrder(CustomIdGenerator())
    action_a = Action('action_a', {})
    plan.plan_action(action_a, [])
    action_b = Action('action_b', {})
    plan.plan_action(action_b, ['a'])
    assert 'a' in plan.actions
    assert 'b' in plan.actions
    assert ['a'] == plan.get_dependency_ids('b')
    plan.remove_action('a')
    assert 'a' not in plan.actions
    assert 'b' in plan.actions
    assert [] == plan.get_dependency_ids('b')
    action_c = Action('action_c', {})
    plan.append_action(action_c)
    assert 'c' in plan.actions
    assert ['b'] == plan.get_dependency_ids('c')


def test_plan_data():
    plan = PartialOrder()
    plan.append_action(Action('action_1', {'param': Frame.worldXY()}))
    other_plan = PartialOrder.from_data(plan.data)
    assert plan.actions.keys() == other_plan.actions.keys()

    # the data attributes point to the same generator,
    # so no testing `append_action` yet
    plan_json = compas.json_dumps(plan.data)
    other_plan_data = compas.json_loads(plan_json)
    other_plan = PartialOrder.from_data(other_plan_data)
    assert plan.actions.keys() == other_plan.actions.keys()
    assert isinstance(other_plan.get_action(1).parameters['param'], Frame)

    # now there are two generators
    plan.append_action(Action('action_2'))
    other_plan.append_action(Action('other_action_2'))
    assert plan.actions.keys() == other_plan.actions.keys()


def test_plan_action():
    plan = PartialOrder()
    action_1_id = plan.plan_action(Action('action_1'), [])
    assert action_1_id == 1
    assert plan.get_action(action_1_id).name == 'action_1'
    action_2_id = plan.plan_action(Action('action_2'), [action_1_id])
    assert action_2_id == 2
    assert plan.get_action(action_2_id).name == 'action_2'
    with pytest.raises(DependencyIdException):
        plan.plan_action(Action('action_impossible'), [5])


def test_append_action():
    plan = PartialOrder()
    action_1_id = plan.append_action(Action('action_1'))
    assert action_1_id == 1
    assert plan.get_action(action_1_id).name == 'action_1'
    assert plan.get_dependency_ids(action_1_id) == []
    action_2_id = plan.append_action(Action('action_2'))
    assert action_2_id == 2
    assert plan.get_action(action_2_id).name == 'action_2'
    assert plan.get_dependency_ids(action_2_id) == [action_1_id]
    with pytest.raises(DependencyIdException):
        plan.plan_action(Action('action_impossible'), [5])


def test_remove_action():
    plan = PartialOrder()
    action_1_id = plan.append_action(Action('action_1'))
    action_2_id = plan.append_action(Action('action_2'))
    action_3_id = plan.append_action(Action('action_3'))
    assert plan.get_dependency_ids(action_2_id) == [action_1_id]
    assert plan.get_dependency_ids(action_3_id) == [action_2_id]
    plan.remove_action(action_2_id)
    assert action_2_id not in plan.actions
    assert plan.get_dependency_ids(action_3_id) == []


def test_check_dependency_ids():
    plan = PartialOrder()
    with pytest.raises(DependencyIdException):
        plan.check_dependency_ids([1])
    id_1 = plan.append_action(Action('action_1'))
    plan.check_dependency_ids([id_1])
    id_2 = plan.append_action(Action('action_2'))
    plan.check_dependency_ids([id_1])
    plan.check_dependency_ids([id_1, id_2])
    with pytest.raises(DependencyIdException):
        plan.check_dependency_ids([3])
    with pytest.raises(DependencyIdException):
        plan.check_dependency_ids([id_1, 5])


def test_check_all_dependency_ids():
    plan = PartialOrder()
    plan.check_all_dependency_ids()
    plan.plan_action(Action('action_1'), [])
    plan.plan_action(Action('action_2'), [1])
    plan.check_all_dependency_ids()
    plan.graph.add_node(3, action=Action('action_3'))
    plan.graph.add_edge(2, 3)
    plan.graph.add_edge(5, 3)
    with pytest.raises(Exception):
        plan.check_all_dependency_ids()
