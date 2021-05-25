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


def test_partial_order():
    partial_order = PartialOrder()
    assert len(partial_order.actions) == 0


def test_partial_order_generator_compatibility():
    action = Action('action', {'param': 3})
    partial_order = PartialOrder()
    action_id = partial_order.add_action(action, [])
    assert action_id == 1
    assert next(partial_order._id_generator) == 2


def test_partial_order_with_custom_id_generator():
    class CustomIdGenerator(object):
        def __init__(self):
            self._generator = count(ord('a'))

        def __next__(self):
            return chr(next(self._generator))

        # alias for ironpython
        next = __next__

    partial_order = PartialOrder(CustomIdGenerator())
    action_a = Action('action_a', {})
    partial_order.add_action(action_a, [])
    action_b = Action('action_b', {})
    partial_order.add_action(action_b, ['a'])
    assert 'a' in partial_order.actions
    assert 'b' in partial_order.actions
    assert ['a'] == partial_order.get_dependency_ids('b')
    partial_order.remove_action('a')
    assert 'a' not in partial_order.actions
    assert 'b' in partial_order.actions
    assert [] == partial_order.get_dependency_ids('b')
    action_c = Action('action_c', {})
    partial_order.append_action(action_c)
    assert 'c' in partial_order.actions
    assert ['b'] == partial_order.get_dependency_ids('c')


def test_partial_order_data():
    partial_order = PartialOrder()
    partial_order.append_action(Action('action_1', {'param': Frame.worldXY()}))
    other_partial_order = PartialOrder.from_data(partial_order.data)
    assert partial_order.actions.keys() == other_partial_order.actions.keys()

    # the data attributes point to the same generator,
    # so no testing `append_action` yet
    partial_order_json = compas.json_dumps(partial_order.data)
    other_partial_order_data = compas.json_loads(partial_order_json)
    other_partial_order = PartialOrder.from_data(other_partial_order_data)
    assert partial_order.actions.keys() == other_partial_order.actions.keys()
    assert isinstance(other_partial_order.get_action(1).parameters['param'], Frame)

    # now there are two generators
    partial_order.append_action(Action('action_2'))
    other_partial_order.append_action(Action('other_action_2'))
    assert partial_order.actions.keys() == other_partial_order.actions.keys()


def test_partial_order_action():
    partial_order = PartialOrder()
    action_1_id = partial_order.add_action(Action('action_1'), [])
    assert action_1_id == 1
    assert partial_order.get_action(action_1_id).name == 'action_1'
    action_2_id = partial_order.add_action(Action('action_2'), [action_1_id])
    assert action_2_id == 2
    assert partial_order.get_action(action_2_id).name == 'action_2'
    with pytest.raises(DependencyIdException):
        partial_order.add_action(Action('action_impossible'), [5])


def test_append_action():
    partial_order = PartialOrder()
    action_1_id = partial_order.append_action(Action('action_1'))
    assert action_1_id == 1
    assert partial_order.get_action(action_1_id).name == 'action_1'
    assert partial_order.get_dependency_ids(action_1_id) == []
    action_2_id = partial_order.append_action(Action('action_2'))
    assert action_2_id == 2
    assert partial_order.get_action(action_2_id).name == 'action_2'
    assert partial_order.get_dependency_ids(action_2_id) == [action_1_id]
    with pytest.raises(DependencyIdException):
        partial_order.add_action(Action('action_impossible'), [5])


def test_remove_action():
    partial_order = PartialOrder()
    action_1_id = partial_order.append_action(Action('action_1'))
    action_2_id = partial_order.append_action(Action('action_2'))
    action_3_id = partial_order.append_action(Action('action_3'))
    assert partial_order.get_dependency_ids(action_2_id) == [action_1_id]
    assert partial_order.get_dependency_ids(action_3_id) == [action_2_id]
    partial_order.remove_action(action_2_id)
    assert action_2_id not in partial_order.actions
    assert partial_order.get_dependency_ids(action_3_id) == []


def test_check_dependency_ids():
    partial_order = PartialOrder()
    with pytest.raises(DependencyIdException):
        partial_order.check_dependency_ids([1])
    id_1 = partial_order.append_action(Action('action_1'))
    partial_order.check_dependency_ids([id_1])
    id_2 = partial_order.append_action(Action('action_2'))
    partial_order.check_dependency_ids([id_1])
    partial_order.check_dependency_ids([id_1, id_2])
    with pytest.raises(DependencyIdException):
        partial_order.check_dependency_ids([3])
    with pytest.raises(DependencyIdException):
        partial_order.check_dependency_ids([id_1, 5])


def test_check_all_dependency_ids():
    partial_order = PartialOrder()
    partial_order.check_all_dependency_ids()
    partial_order.add_action(Action('action_1'), [])
    partial_order.add_action(Action('action_2'), [1])
    partial_order.check_all_dependency_ids()
    partial_order.graph.add_node(3, action=Action('action_3'))
    partial_order.graph.add_edge(2, 3)
    partial_order.graph.add_edge(5, 3)
    with pytest.raises(Exception):
        partial_order.check_all_dependency_ids()
