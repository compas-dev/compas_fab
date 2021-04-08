from itertools import count

import compas
import pytest

from compas.geometry import Frame
from compas_fab.datastructures import Action
from compas_fab.datastructures import IntegerIdGenerator
from compas_fab.datastructures import Plan
from compas_fab.datastructures import PlannedAction
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


def test_planned_action_data():
    action_id = 1
    action = Action('action', {'param': 'hi there'})
    dependency_ids = []
    planned_action = PlannedAction(action_id, action, dependency_ids)
    other_planned_action = PlannedAction.from_data(planned_action.data)
    assert planned_action.id == other_planned_action.id
    assert planned_action.action == other_planned_action.action
    assert planned_action.dependency_ids == other_planned_action.dependency_ids

    pa_json = compas.json_dumps(planned_action.data)
    other_pa_data = compas.json_loads(pa_json)
    other_pa = PlannedAction.from_data(other_pa_data)
    assert other_pa.action.parameters['param'] == 'hi there'


def test_integer_id_generator():
    generator = IntegerIdGenerator()
    assert next(generator) == 1
    assert next(generator) == 2
    generator_json = compas.json_dumps(generator.data)
    generator_data = compas.json_loads(generator_json)
    generator_reincarnate = IntegerIdGenerator.from_data(generator_data)
    assert next(generator_reincarnate) == 3


def test_plan():
    plan = Plan()
    assert len(plan.planned_actions) == 0


def test_plan_generator_compatibility():
    action = Action('action', {'param': 3})
    pa = PlannedAction(1, action, [])
    plan_a = Plan([pa])
    assert plan_a.planned_actions[1].action.parameters['param'] == 3
    assert next(plan_a._id_generator) == 2

    pa = PlannedAction('a', action, [])
    with pytest.raises(Exception):
        _ = Plan([pa])


def test_plan_with_custom_id_generator():
    class CustomIdGenerator(object):
        def __init__(self):
            self._generator = count(ord('a'))

        def __next__(self):
            return chr(next(self._generator))

    plan = Plan([], CustomIdGenerator())
    action_a = Action('action_a', {})
    plan.plan_action(action_a, [])
    action_b = Action('action_b', {})
    plan.plan_action(action_b, ['a'])
    assert 'a' in plan.planned_actions
    assert 'b' in plan.planned_actions
    assert {'a'} == plan.planned_actions['b'].dependency_ids
    plan.remove_action_by_id('a')
    assert 'a' not in plan.planned_actions
    assert 'b' in plan.planned_actions
    assert set() == plan.planned_actions['b'].dependency_ids
    action_c = Action('action_c', {})
    plan.append_action(action_c)
    assert 'c' in plan.planned_actions
    assert {'b'} == plan.planned_actions['c'].dependency_ids


def test_plan_data():
    plan = Plan()
    plan.append_action(Action('action_1'))
    other_plan = Plan.from_data(plan.data)
    assert plan.planned_actions.keys() == other_plan.planned_actions.keys()

    # the data attributes point to the same generator,
    # so no testing `append_action` yet
    plan_json = compas.json_dumps(plan.data)
    other_plan_data = compas.json_loads(plan_json)
    other_plan = Plan.from_data(other_plan_data)
    assert plan.planned_actions.keys() == other_plan.planned_actions.keys()

    # now there are two generators
    plan.append_action(Action('action_2'))
    other_plan.append_action(Action('other_action_2'))
    assert plan.planned_actions.keys() == other_plan.planned_actions.keys()


def test_plan_action():
    plan = Plan()
    action_1_id = plan.plan_action(Action('action_1'), [])
    assert action_1_id == 1
    assert plan.planned_actions[action_1_id].action.name == 'action_1'
    action_2_id = plan.plan_action(Action('action_2'), [action_1_id])
    assert action_2_id == 2
    assert plan.planned_actions[action_2_id].action.name == 'action_2'
    with pytest.raises(DependencyIdException):
        plan.plan_action(Action('action_impossible'), [5])


def test_append_action():
    plan = Plan()
    action_1_id = plan.append_action(Action('action_1'))
    assert action_1_id == 1
    assert plan.planned_actions[action_1_id].action.name == 'action_1'
    assert plan.planned_actions[action_1_id].dependency_ids == set()
    action_2_id = plan.append_action(Action('action_2'))
    assert action_2_id == 2
    assert plan.planned_actions[action_2_id].action.name == 'action_2'
    assert plan.planned_actions[action_2_id].dependency_ids == {action_1_id}
    with pytest.raises(DependencyIdException):
        plan.plan_action(Action('action_impossible'), [5])


def test_remove_action():
    plan = Plan()
    action_1_id = plan.append_action(Action('action_1'))
    action_2_id = plan.append_action(Action('action_2'))
    action_3_id = plan.append_action(Action('action_3'))
    assert plan.planned_actions[action_2_id].dependency_ids == {action_1_id}
    assert plan.planned_actions[action_3_id].dependency_ids == {action_2_id}
    plan.remove_action_by_id(action_2_id)
    assert action_2_id not in plan.planned_actions
    assert plan.planned_actions[action_3_id].dependency_ids == set()


def test_check_dependency_ids():
    plan = Plan()
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
    plan = Plan()
    plan.check_all_dependency_ids()
    pa1 = PlannedAction(1, Action('action_1'), [])
    pa2 = PlannedAction(2, Action('action_2'), [1])
    plan.planned_actions[1] = pa1
    plan.planned_actions[2] = pa2
    plan.check_all_dependency_ids()
    pa3 = PlannedAction(3, Action('action_3'), [2, 5])
    plan.planned_actions[3] = pa3
    with pytest.raises(DependencyIdException):
        plan.check_all_dependency_ids()
    with pytest.raises(DependencyIdException):
        _ = Plan([pa1, pa2, pa3])


def test_check_for_cycles():
    plan = Plan()
    plan.append_action(Action('action_1'))
    plan.planned_actions[1].dependency_ids.add(1)
    with pytest.raises(Exception):
        plan.check_for_cycles()

    plan = Plan()
    for i in range(5):
        plan.append_action(Action('action_{}'.format(i + 1)))
    plan.check_for_cycles()
    plan.plan_action(Action('action_6'), [])
    plan.check_for_cycles()
    plan.planned_actions[4].dependency_ids.add(2)
    plan.check_for_cycles()
    plan.planned_actions[1].dependency_ids.add(3)
    with pytest.raises(Exception):
        plan.check_for_cycles()
    plan.planned_actions[1].dependency_ids.discard(3)
    plan.planned_actions[1].dependency_ids.add(2)
    with pytest.raises(Exception):
        plan.check_for_cycles()


def test_linear_sort():
    plan = Plan()
    ids = []
    for i in range(5):
        pa_id = plan.append_action(Action('action_{}'.format(i + 1)))
        ids.append(pa_id)
    sorted_actions = plan.linear_sort()
    assert [pa.id for pa in sorted_actions] == ids

    plan = Plan()
    assert plan.linear_sort() == []

    pa1_id = plan.plan_action(Action('action_1'), [])
    pa2_id = plan.plan_action(Action('action_2'), [])
    plan.plan_action(Action('action_3'), [pa1_id, pa2_id])
    sorted_actions = plan.linear_sort()
    sorted_ids = [pa.id for pa in sorted_actions]
    assert sorted_ids == [1, 2, 3] or sorted_ids == [2, 1, 3]
