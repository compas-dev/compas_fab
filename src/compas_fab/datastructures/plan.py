from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from collections import OrderedDict
from itertools import count

from compas.utilities import DataEncoder
from compas.utilities import DataDecoder


__all__ = [
    'Action',
    'IntegerIdGenerator',
    'Plan',
    'PlannedAction',
]


class IntegerIdGenerator(object):
    def __init__(self, start_value=1):
        self._generator = count(start_value)

    def __next__(self):
        yield next(self._generator)


class Plan(object):
    def __init__(self, planned_actions=None, id_generator=None):
        self.planned_actions = OrderedDict({pa.id: pa for pa in planned_actions})
        if id_generator is None:
            start_value = max(self.planned_actions.keys()) + 1 if self.planned_actions else 1
            id_generator = IntegerIdGenerator(start_value)
        self._id_generator = id_generator

    def plan_action(self, action, dependency_ids, namespace=None):
        action_id = self._get_next_action_id()
        planned_action = PlannedAction(action_id, action, dependency_ids, namespace)
        self.planned_actions[action_id] = planned_action
        return action_id

    def append_action(self, action, namespace=None):
        dependency_ids = set()
        if self.planned_actions:
            last_action_id = self._get_last_action_id()
            dependency_ids = {last_action_id}
        return self.plan_action(action, dependency_ids, namespace)

    def remove_action_by_id(self, action_id):
        planned_action = self.planned_actions.pop(action_id)
        for pa in self.planned_actions.values():
            pa.dependency_ids.discard(action_id)
        return planned_action.action

    def _get_last_action_id(self):
        last_action_id, last_action = self.planned_actions.popitem()
        self.planned_actions[last_action_id] = last_action
        return last_action_id

    def _get_next_action_id(self):
        return next(self._id_generator)

    def get_namespaces(self):
        return set(a.namespace for a in self.planned_actions.values())

    @property
    def data(self):
        return dict(
            planned_actions=[p.to_data() for p in self.planned_actions.values()]
        )

    @classmethod
    def from_data(cls, data):
        return cls([PlannedAction.from_data(d) for d in data['planned_actions']])

    def to_data(self):
        return self.data


class PlannedAction(object):
    def __init__(self, action_id, action, dependency_ids, namespace=None):
        self.id = action_id
        self.action = action
        self.dependency_ids = set(dependency_ids)
        self.namespace = namespace

    def __str__(self):
        return 'PlannedAction<id={}, action={}, namespace={}>'.format(self.id, self.action, self.namespace)

    @property
    def data(self):
        return dict(
            id=self.id,
            action=self.action.to_data(),
            dependency_ids=self.dependency_ids,
            namespace=self.namespace,
        )

    @classmethod
    def from_data(cls, data):
        return cls(data['id'], Action.from_data(data['action']), data['dependency_ids'], data['namespace'])

    def to_data(self):
        return self.data


class Action(object):
    def __init__(self, name, parameters=None):
        self.name = name
        self.parameters = parameters or {}

    def __str__(self):
        return 'Action<name={}>'.format(self.name)

    @property
    def data(self):
        return dict(
            name=self.name,
            parameters={param_key: DataEncoder().default(p) if hasattr(p, 'to_data') else p for param_key, p in
                        self.parameters.items()},
        )

    @classmethod
    def from_data(cls, data):
        return cls(data['name'],
                   {param_key: DataDecoder().object_hook(p) if hasattr(p, '__iter__') else p for param_key, p in
                    data['parameters'].items()})

    def to_data(self):
        return self.data
