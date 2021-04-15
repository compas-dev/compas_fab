from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import threading
from collections import OrderedDict
from copy import deepcopy
from itertools import count
from networkx import all_topological_sorts
from networkx import find_cycle
from networkx import lexicographical_topological_sort
from networkx import NetworkXNoCycle

import compas
from compas.base import Base
from compas.datastructures import Datastructure
from compas.datastructures import Graph

__all__ = [
    'Action',
    'DependencyIdException',
    'IntegerIdGenerator',
    'Plan',
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
    """Data structure extending :class:`compas.datastructures.Graph` for creating
    and maintaining a partially ordered plan (directed acyclic graph).
    The content of any event of the plan is contained in an
    :class:`compas_fab.datastructures.Action`.  The dependency ids of a planned
    action can be thought of as pointers to the parents of that planned action.
    While actions can be added and removed using the methods of
    :attr:`compas_fab.datastructures.Plan.graph`, it is strongly recommended
    that the methods ``plan_action``, ``append_action`` and ``remove_action``
    are used instead.

    Attributes
    ----------
    graph : :class:`compas.datastructures.Graph
    id_generator : Generator[Hashable, None, None]
        Object which generates keys (via ``next()``) for
        :class:`compas_fab.datastructures.Action`s added using this object's
        methods.  Defaults to :class:`compas_fab.datastructures.IntegerIdGenerator`.
    """
    def __init__(self, id_generator=None):
        super(Plan, self).__init__()
        self.graph = Graph()
        self.graph.node = OrderedDict()
        self._id_generator = id_generator or IntegerIdGenerator()

    @property
    def networkx(self):
        """A new NetworkX DiGraph instance from ``graph``."""
        return self.graph.to_networkx()

    @property
    def actions(self):
        """A dictionary of id-:class:`compas_fab.datastructures.Action` pairs."""
        return {action_id: self.get_action(action_id) for action_id in self.graph.nodes()}

    def get_action(self, action_id):
        """Gets the action for the associated ``action_id``

        Parameters
        ----------
        action_id : hashable

        Returns
        -------
        :class:`compas_fab.datastructures.Action`
        """
        action = self.graph.node_attribute(action_id, 'action')
        if action is None:
            raise Exception("Action with id {} not found".format(action_id))
        return action

    def remove_action(self, action_id):
        action = self.get_action(action_id)
        self.graph.delete_node(action_id)
        return action

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
        self.graph.add_node(action_id, action=action)
        for dependency_id in dependency_ids:
            self.graph.add_edge(dependency_id, action_id)
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
        if self.graph.node:
            last_action_id = self._get_last_action_id()
            dependency_ids = {last_action_id}
        return self.plan_action(action, dependency_ids)

    def _get_last_action_id(self):
        last_action_id, last_action_attrs = self.graph.node.popitem()
        self.graph.node[last_action_id] = last_action_attrs
        return last_action_id

    def _get_next_action_id(self):
        return next(self._id_generator)

    def get_dependency_ids(self, action_id):
        """Return the identifiers of actions upon which the action with id ``action_id`` is dependent.

        Parameters
        ----------
        action_id : hashable
            The identifier of the action.

        Returns
        -------
        :obj:`list`
            A list of action identifiers.

        """
        return self.graph.neighbors_in(action_id)

    def check_dependency_ids(self, dependency_ids, action_id=None):
        """Checks whether the given dependency ids exist in the plan.

        Parameters
        ----------
        dependency_ids : set or list
            The dependency ids to be validated.
        action_id : hashable
            The id of the associated action.  Used only in
            the error message.  Defaults to ``None``.

        Raises
        ------
        :class:`compas_fab.datastructures.DependencyIdException`
        """
        dependency_ids = set(dependency_ids)
        if not dependency_ids.issubset(self.graph.node):
            invalid_ids = dependency_ids.difference(self.graph.node)
            raise DependencyIdException(invalid_ids, action_id)

    def check_all_dependency_ids(self):
        """Checks whether the dependency ids of all the planned actions
        are ids of planned actions in the plan.

        Raises
        ------
        :class:`compas_fab.datastructures.DependencyIdException`
        """
        for action_id in self.actions:
            self.check_dependency_ids(self.get_dependency_ids(action_id), action_id)

    def check_for_cycles(self):
        """"Checks whether cycles exist in the dependency graph."""
        try:
            cycle = find_cycle(self.networkx)
        except NetworkXNoCycle:
            return
        raise Exception("Cycle found with edges {}".format(cycle))  # !!!

    def get_linear_sort(self):
        """Sorts the planned actions linearly respecting the dependency ids.

        Returns
        -------
        :obj:`list` of :class:`compas_fab.datastructure.Action`
        """
        self.check_for_cycles()
        return [self.get_action(action_id) for action_id in lexicographical_topological_sort(self.networkx)]

    def get_all_linear_sorts(self):
        """Gets all possible linear sorts respecting the dependency ids.

        Returns
        -------
        :obj:`list` of :obj:`list of :class:`compas_fab.datastructure.Action`
        """
        self.check_for_cycles()
        return [[self.get_action(action_id) for action_id in sorting] for sorting in all_topological_sorts(self.networkx)]

    @property
    def data(self):
        return {
            'graph': self.graph,
            'id_generator': self._id_generator,
        }

    @data.setter
    def data(self, data):
        graph = data['graph']
        graph.node = OrderedDict(graph.node)
        self.graph = graph
        self._id_generator = data['id_generator']


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
