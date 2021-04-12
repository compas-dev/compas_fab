from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import threading
from collections import OrderedDict
from copy import deepcopy
from itertools import count

import compas
from compas.base import Base
from compas.datastructures import Datastructure


__all__ = [
    'Action',
    'DependencyIdException',
    'IntegerIdGenerator',
    'Plan',
    'PlannedAction',
]


class IntegerIdGenerator(Base):
    """Generator object yielding integers sequentially in a thread safe manner.

    Parameters
    ----------
    start_value : :obj:`int`
        First value to be yielded by the generator.
    """
    def __init__(self, start_value=1):
        super(IntegerIdGenerator, self).__init__()
        self.last_generated = start_value - 1
        self._lock = threading.Lock()
        self._generator = count(start_value)

    def __next__(self):
        with self._lock:
            self.last_generated = next(self._generator)
            return self.last_generated

    # alias for ironpython
    next = __next__

    @property
    def data(self):
        return {
            'start_value': self.last_generated + 1
        }

    def to_data(self):
        return self.data

    @classmethod
    def from_data(cls, data):
        return cls(data['start_value'])

    @classmethod
    def from_json(cls, filepath):
        data = compas.json_load(filepath)
        return cls.from_data(data)

    def to_json(self, filepath):
        compas.json_dump(self.data, filepath)


class DependencyIdException(Exception):
    """Indicates invalid ids given as dependencies."""
    def __init__(self, invalid_ids, pa_id=None):
        message = self.compose_message(invalid_ids, pa_id)
        super(DependencyIdException, self).__init__(message)

    @staticmethod
    def compose_message(invalid_ids, pa_id):
        if pa_id:
            return 'Planned action {} has invalid dependency ids {}'.format(pa_id, invalid_ids)
        return 'Found invalid dependency ids {}'.format(invalid_ids)


class Plan(Datastructure):
    """Data structure for holding the information of a partially ordered plan
    (a directed acyclic graph).  The content of any event of the plan is contained
    in an :class:`compas_fab.datastructures.Action`. An event is scheduled and
    added to the plan through a :class:`compas_fab.datastructures.PlannedAction`.
    The dependency ids of a planned action can be thought of as pointers to the
    parents of that planned action.

    Parameters
    ----------
    planned_actions : list of :class:`compas_fab.datastructures.PlannedAction`
        The planned actions will be stored as an ordered dictionary, so their
        ids should be distinct.  Defaults to an empty list.
    id_generator : Generator[Hashable, None, None]
        Object which generates keys (via ``next()``) for
        :class:`compas_fab.datastructures.Action`s added using this object's
        methods.  Defaults to :class:`compas_fab.datastructures.IntegerIdGenerator`.
    """
    def __init__(self, planned_actions=None, id_generator=None):
        super(Plan, self).__init__()
        planned_actions = planned_actions or []
        self.planned_actions = planned_actions
        if id_generator is None:
            try:
                start_value = max(self.planned_actions.keys()) + 1 if self.planned_actions else 1
            except Exception:
                raise Exception('Given ids not compatible with default id_generator.')
            id_generator = IntegerIdGenerator(start_value)
        self._id_generator = id_generator

    @property
    def planned_actions(self):
        return self._planned_actions

    @planned_actions.setter
    def planned_actions(self, planned_actions):
        self._planned_actions = OrderedDict({pa.id: pa for pa in planned_actions})
        self.check_all_dependency_ids()

    def plan_action(self, action, dependency_ids):
        """Adds the action to the plan with the given dependencies,
        and generates an id for the newly planned action.

        Parameters
        ----------
        action : :class:`comaps_fab.datastructures.Action`
            The action to be added to the plan.
        dependency_ids : set or list
            The keys of the already planned actions that the new action
            is dependent on.

        Returns
        -------
            The id of the newly planned action.
        """
        self.check_dependency_ids(dependency_ids)
        action_id = self._get_next_action_id()
        planned_action = PlannedAction(action_id, action, dependency_ids)
        self.planned_actions[action_id] = planned_action
        return action_id

    def append_action(self, action):
        """Adds the action to the plan dependent on the last action added
        to the plan, and generates an id for this newly planned action.

        Parameters
        ----------
        action : :class:`comaps_fab.datastructures.Action`
            The action to be added to the plan.

        Returns
        -------
            The id of the newly planned action.
        """
        dependency_ids = set()
        if self.planned_actions:
            last_action_id = self._get_last_action_id()
            dependency_ids = {last_action_id}
        return self.plan_action(action, dependency_ids)

    def remove_action_by_id(self, action_id):
        """Removes the action with the given id from the plan and all its
        dependencies.

        Parameters
        ----------
        action_id : Hashable
            Id of the planned action to be removed.

        Returns
        -------
        :class:`compas_fab.datastructure.Action`
            The action of the removed :class:`compas_fab.datastructure.PlannedAction`
        """
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

    def check_dependency_ids(self, dependency_ids, planned_action_id=None):
        """Checks whether the given dependency ids exist in the plan.

        Parameters
        ----------
        dependency_ids : set or list
            The dependency ids to be validated.
        planned_action_id : Hashable
            The id of the associated planned action.  Used only in
            the error message.  Defaults to ``None``.

        Raises
        ------
        :class:`compas_fab.datastructures.DependencyIdException`
        """
        dependency_ids = set(dependency_ids)
        if not dependency_ids.issubset(self.planned_actions):
            invalid_ids = dependency_ids.difference(self.planned_actions)
            raise DependencyIdException(invalid_ids, planned_action_id)

    def check_all_dependency_ids(self):
        """Checks whether the dependency ids of all the planned actions
        are ids of planned actions in the plan.

        Raises
        ------
        :class:`compas_fab.datastructures.DependencyIdException`
        """
        for pa_id, planned_action in self.planned_actions.items():
            self.check_dependency_ids(planned_action.dependency_ids, pa_id)

    def check_for_cycles(self):
        """"Checks whether cycles exist in the dependency graph."""
        self.check_all_dependency_ids()

        def helper(cur, v, r):
            v[cur] = True
            r[cur] = True
            for dep_id in self.planned_actions[cur].dependency_ids:
                if not v[dep_id]:
                    helper(dep_id, v, r)
                elif r[dep_id]:
                    raise Exception("Cycle found with action ids {}".format([pa_id for pa_id, seen in r.items() if seen]))
            r[cur] = False

        visited = {pa_id: False for pa_id in self.planned_actions}
        rec_dict = {pa_id: False for pa_id in self.planned_actions}

        for pa_id in self.planned_actions:
            if not visited[pa_id]:
                helper(pa_id, visited, rec_dict)

    def linear_sort(self):
        """Sorts the planned actions linearly respecting the dependency ids.

        Returns
        -------
        :obj:`list` of :class:`compas_fab.datastructure.Action`
        """
        self.check_for_cycles()

        def helper(s, v, cur_id):
            v.add(cur_id)
            action = self.planned_actions[cur_id]
            for dep_id in action.dependency_ids:
                if dep_id not in v:
                    helper(s, v, dep_id)
            s.append(action)

        stack = []
        visited = set()

        for action_id in self.planned_actions:
            if action_id not in visited:
                helper(stack, visited, action_id)

        return stack

    @property
    def data(self):
        return dict(
            planned_actions=list(self.planned_actions.values()),
            id_generator=self._id_generator,
        )

    @data.setter
    def data(self, data):
        self.planned_actions = data['planned_actions']
        self._id_generator = data['id_generator']

    @classmethod
    def from_data(cls, data):
        return cls(**data)


class PlannedAction(Base):
    """Represents an action which has been scheduled in a plan.

    Parameters
    ----------
    action_id : Hashable
        An identifier of the action.  Used by other actions and the plan
        it is associated with.
    action : :class:`compas_fab.datastructures.Action`
        The action to be planned.
    dependency_ids : set or list
        The ids of the actions upon which `action` is dependent.
    """
    def __init__(self, action_id, action, dependency_ids):
        super(PlannedAction, self).__init__()
        self.id = action_id
        self.action = action
        self.dependency_ids = dependency_ids

    @property
    def dependency_ids(self):
        return self._dependency_ids

    @dependency_ids.setter
    def dependency_ids(self, value):
        self._dependency_ids = set(value)

    def __str__(self):
        return 'PlannedAction<id={}, action={}>'.format(self.id, self.action)

    @property
    def data(self):
        return dict(
            action_id=self.id,
            action=self.action,
            # sets are not json serializable
            dependency_ids=list(self.dependency_ids),
        )

    @data.setter
    def data(self, data):
        self.id = data['action_id']
        self.action = data['action']
        self.dependency_ids = data['dependency_ids']

    @classmethod
    def from_data(cls, data):
        return cls(**data)

    def to_data(self):
        return self.data

    @classmethod
    def from_json(cls, filepath):
        data = compas.json_load(filepath)
        return cls.from_data(data)

    def to_json(self, filepath):
        compas.json_dump(self.data, filepath)

    def copy(self, cls=None):
        if not cls:
            cls = type(self)
        return cls.from_data(deepcopy(self.data))


class Action(Base):
    """Abstract representation of an event independent of its timing.

    Parameters
    ----------
    name : :obj:`str`
        The name of the action.
    parameters : dict
        Any other content associated to the action housed in key-value pairs.
    """
    def __init__(self, name, parameters=None):
        super(Action, self).__init__()
        self.name = name
        self.parameters = parameters or {}

    def __str__(self):
        return 'Action<name={}>'.format(self.name)

    @property
    def data(self):
        return dict(
            name=self.name,
            parameters=self.parameters,
        )

    @data.setter
    def data(self, data):
        self.name = data['name']
        self.parameters = data['parameters']

    @classmethod
    def from_data(cls, data):
        return cls(**data)

    def to_data(self):
        return self.data

    @classmethod
    def from_json(cls, filepath):
        data = compas.json_load(filepath)
        return cls.from_data(data)

    def to_json(self, filepath):
        compas.json_dump(self.data, filepath)

    def copy(self, cls=None):
        if not cls:
            cls = type(self)
        return cls.from_data(deepcopy(self.data))
