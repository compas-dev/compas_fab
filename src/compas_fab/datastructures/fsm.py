'''
Created on 17 May 2017

@author: romana
'''

"""
An implementation of the Finite State Machine.
Overview of classes:
    
"""

# MultiDiGraph


class FSMError(Exception):
    """Base FSM exception."""
    pass

class TransitionError(FSMError):
    """Transition exception."""
    pass

class StateError(FSMError):
    """State manipulation error."""
    pass


class FiniteStateMachine(object):
    """An implementation of a generic Finite State Machine. 
    
    Transition -- A transition is a set of actions to be executed when a 
    condition is fulfilled or when an event is received. 
    
    """

    DOT_ATTRIBUTES = {
        'directed': 'True',
        'strict': 'False',
        'rankdir': 'LR',
        'ratio': '0.3'
    }

    def __init__(self, name, default_transition=True):
        self.name = name
        self.inputs = []
        self.states = []
        self.initial_state = None
        self.current_state = None
        self.accepting_states = []
    
    def add_state(self, name, initial=False, accepting=False, **kwargs):
        state = State(name, **kwargs)
        self.states.append(state)
        if initial:
            self.initial_state = state
            self.current_state = self.initial_state
        if accepting:
            self.accepting_states.append(state)
        return state

    def add_transition(self, src_state, dst_state, input_value=None, action=None):
        src_state[action] = dst_state
        src_state.output_values[input_value] = action
        dst_state.input_values[input_value] = action
    
    @property
    def transitions(self):
        """Get transitions from states.
        
        Returns:
            List of three element tuples each consisting of
            (source state, action, destination state)
        """
        transitions = []
        for src_state in self.states:
            for action, dst_state in src_state.items():
                transitions.append((src_state, action, dst_state))
        return transitions
    
    def reset(self):
        self.current_state = self.initial_state
        
    def next(self, input_value):
        """Transition to the next state."""
        current = self.current_state
        if current is None:
            raise TransitionError('Current state not set.')
        
        print current.output_values
        action = current.output_values[input_value]
        dst_state = current[action]
        if dst_state is None: 
            raise TransitionError('Cannot transition from state %r on input %r.' % (current.name, input_value))
        else:
            self.current_state = dst_state

    def process(self, input_values):
        """This is the main method that is called to process input. This may
        cause the FSM to change state and call an action. This method calls
        transition() to find the action and next_state associated with the
        input_value and current_state. If the action is None then the action
        is not called and only the current state is changed."""
        self.reset()
        for value in input_values:
            self.next(value)
    
    def graph(self, textwidth=12):
        """Generate a DOT graph with graphviz."""
        
        import textwrap
        from graphviz import Digraph
        
        def format_label(label):
            return "\n".join(textwrap.wrap(label, textwidth))
        
        fsm_graph = Digraph(self.name)
        fsm_graph.attr('graph', **self.DOT_ATTRIBUTES)
        fsm_graph.attr('node', State.DOT_ATTRIBUTES_DEFAULT)
        
        for state in self.states:
            shape = state.DOT_ATTRIBUTES_DEFAULT['shape']
            if id(state) in [id(s) for s in self.accepting_states]:
                shape = state.DOT_ATTRIBUTES_ACCEPTING
            if id(state) == id(self.current_state):
                fsm_graph.node(state.name, format_label(state.name), shape=shape, **state.DOT_ATTRIBUTES_CURRENT)
            else:  
                fsm_graph.node(state.name, format_label(state.name), shape=shape)
    
        fsm_graph.node('null', ' ', shape='plaintext')
        fsm_graph.edge('null', self.initial_state.name)
    
        for src, action, dst in self.transitions:
            label = ""
            if action:
                label = str(action)
                for k, v in dst.input_values.iteritems():
                    if v == action:
                        label += ' / %s' % k
                        break
            fsm_graph.edge(src.name, dst.name, label=format_label(label))

        return fsm_graph
                
    def show(self, filename="tmp.png"):
        graph = self.graph()
        graph.render(filename, view=True)
        
            

class State(dict):
    
    DOT_ATTRIBUTES_DEFAULT = { 'shape': 'circle', 'height': '1.2'}
    DOT_ATTRIBUTES_CURRENT = {'style': 'filled', 'fillcolor': '#ededed'}
    DOT_ATTRIBUTES_ACCEPTING = 'doublecircle'

    def __init__(self, name, output=None):
        """A class representing a state which can be used in a finite state
        machine of any type.
        
        State -- A state is a description of the status of a system that is 
        waiting to execute a transition according to an input.
        """
        
        dict.__init__(self)
        self.name = name
        self.output_values = {} # output_values[value] = action
        self.input_values = {} # input_values[value] = action

    def __getitem__(self, action):
        """Make a transition to the next state."""
        next_state = dict.__getitem__(self, action)
        return next_state

    def __setitem__(self, action, next_state):
        """Set a transition to a new state."""
        if not isinstance(next_state, State):
            raise StateError('A state must transition to another state, got %r instead.' % next_state)
        dict.__setitem__(self, action, next_state)
    
    def __repr__(self):
        return '<%r %s @ 0x%x>' % (self.name, self.__class__.__name__, id(self))


if __name__ == "__main__":
    
    fsm = FiniteStateMachine('finite state machine')
    
    l0 = fsm.add_state('LR_0', initial=True, accepting=True)
    l1 = fsm.add_state('LR_1')
    l2 = fsm.add_state('LR_2')
    l3 = fsm.add_state('LR_3', accepting=True)
    l4 = fsm.add_state('LR_4', accepting=True)
    l5 = fsm.add_state('LR_5')
    l6 = fsm.add_state('LR_6')
    l7 = fsm.add_state('LR_7')
    l8 = fsm.add_state('LR_8', accepting=True)
    
    fsm.add_transition(l0, l2, action='SS(B)')
    fsm.add_transition(l0, l1, action='SS(S)')
    fsm.add_transition(l1, l3, action='S($end)')
    fsm.add_transition(l2, l6, action='SS(b)')
    fsm.add_transition(l2, l5, action='SS(a)')
    fsm.add_transition(l2, l4, action='S(A)')
    fsm.add_transition(l5, l7, action='S(b)')
    fsm.add_transition(l5, l5, action='S(a)')
    fsm.add_transition(l6, l6, action='S(b)')
    fsm.add_transition(l6, l5, action='S(a)')
    fsm.add_transition(l7, l8, action='S(b)')
    fsm.add_transition(l7, l5, action='S(a)')
    fsm.add_transition(l8, l6, action='S(b)')
    fsm.add_transition(l8, l5, action='S(a)')

    fsm.show()


    

    