'''
Created on 17 May 2017

@author: romana
'''

"""
An implementation of the Finite State Machine.
Overview of classes:
    State -- A state is a description of the status of a system that is 
    waiting to execute a transition.
    Transition -- A transition is a set of actions to be executed when a 
    condition is fulfilled or when an event is received. 
    FiniteStateMachine -- a semiautomaton base for all following classes.
        This class implements the process() method which takes an iterator
        as input and processes it.
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


class FiniteStateMachine(object):
    """An implementation of a generic Finite State Machine. 
    
    This class implements the process() method which takes an iterator
    as input and processes it.
    """

    DOT_ATTRIBUTES = {
        'directed': True,
        'strict': False,
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

    def add_transition(self, input, src_state, action, dst_state):
        src_state[action] = dst_state
        src_state.output_values[action] = input
        dst_state.input_values[action] = input
    
    @property
    def transitions(self):
        """Get transitions from states.
        
        Returns:
            List of three element tuples each consisting of
            (source state, input, destination state)
        """
        transitions = []
        for src_state in self.states:
            for action, dst_state in src_state.items():
                transitions.append((src_state, action, dst_state))
        return transitions
        
    
    def transition(self, input_value):
        """Transition to the next state."""
        current = self.current_state
        if current is None:
            raise TransitionError('Current state not set.')

        destination_state = current.get(input_value, current.default_transition)
        if destination_state is None: 
            raise TransitionError('Cannot transition from state %r on input %r.' % (current.name, input_value))
        else:
            self.current_state = destination_state

    def reset(self):
        """Enter the Finite State Machine."""
        self.current_state = self.initial_state

    def process(self, input_data):
        """This is the main method that is called to process input. This may
        cause the FSM to change state and call an action. This method calls
        get_transition() to find the action and next_state associated with the
        input_symbol and current_state. If the action is None then the action
        is not called and only the current state is changed. This method
        processes one complete input symbol. You can process a list of symbols
        (or a string) by calling process_list(). """
        self.reset()
        for item in input_data:
            self.transition(item)
        
        """
        self.input_symbol = input_symbol
        (self.action, self.next_state) = self.get_transition (self.input_symbol, self.current_state)
        if self.action is not None:
            self.action (self)
        self.current_state = self.next_state
        self.next_state = None
        """
    
    def graph(self, textwidth=12):
        """Generate a DOT graph with pygraphviz."""
    
        import pygraphviz as pgv
        import textwrap
        
        fsm_graph = pgv.AGraph(title=self.name, **self.DOT_ATTRIBUTES)
        fsm_graph.node_attr.update(State.DOT_ATTRIBUTES_DEFAULT)
        
        def format_label(label):
            return "\n".join(textwrap.wrap(label, textwidth))
    
        #for state in [self.initial_state] + self.states:
        for state in self.states:
            shape = state.DOT_ATTRIBUTES_DEFAULT['shape']
            if id(state) in [id(s) for s in self.accepting_states]:
                shape = state.DOT_ATTRIBUTES_ACCEPTING
            if id(state) == id(self.current_state):
                fsm_graph.add_node(n=state.name, shape=shape, label=format_label(state.name), **state.DOT_ATTRIBUTES_CURRENT)
            else:  
                fsm_graph.add_node(n=state.name, shape=shape, label=format_label(state.name))
    
        fsm_graph.add_node('null', shape='plaintext', label=' ')
        fsm_graph.add_edge('null', self.initial_state.name)
    
        for src, action, dst in self.transitions:
            if action:
                label = str(action)
            else:
                label = ""
            if dst.input_values[action]:
                label += ' / %s' % dst.input_values[action]
            fsm_graph.add_edge(src.name, dst.name, label=format_label(label))
        """
        for state in self.states:
            if state.default_transition is not None:
                fsm_graph.add_edge(state.name, state.default_transition.name, label='else')
        """
        return fsm_graph
    
    def draw_graph(self, filename="tmp.png"):
        graph = self.graph()
        graph.draw(filename, prog='dot')
    
    def show_graph(self, filename="tmp.png"):
        from PIL import Image
        self.draw_graph(filename)
        Image.open(filename).show()
        
            

class State(dict):
    
    """State class."""

    DOT_ATTRIBUTES_DEFAULT = { 'shape': 'circle', 'height': '1.2'}
    DOT_ATTRIBUTES_CURRENT = {'style': 'filled', 'fillcolor': '#ededed'}
    DOT_ATTRIBUTES_ACCEPTING = 'doublecircle'

    def __init__(self, name, output=None):
        """A class representing a state which can be used in a finite state
        machine of any type.
        # , on_entry=NOOP, on_exit=NOOP, on_input=NOOP_ARG, on_transition=NOOP_ARG, default_transition=None
        """
        dict.__init__(self)
        self.name = name
        
        self.output_values = {} # output_values[action] = value
        self.input_values = {} # input_values[action] = value
        """
        self.entry_action = on_entry
        self.exit_action = on_exit
        self.input_action = on_input
        self.transition_action = on_transition
        self.default_transition = default_transition
        """

    def __getitem__(self, action):
        """Make a transition to the next state."""
        next_state = dict.__getitem__(self, action)
        #self.input_action(input_value)
        #self.exit_action()
        #self.transition_action(next_state)
        #next_state.entry_action()
        return next_state

    def __setitem__(self, action, next_state):
        """Set a transition to a new state."""
        if not isinstance(next_state, State):
            raise StateError('A state must transition to another state, got %r instead.' % next_state)
        #if isinstance(action, tuple):
        #    action, output = action
        #    self.output_values.append((action, output))
        dict.__setitem__(self, action, next_state)

    def __repr__(self):
        """Represent the object in a string."""
        return '<%r %s @ 0x%x>' % (self.name, self.__class__.__name__, id(self))


class Transducer(FiniteStateMachine):
    
    """A semiautomaton transducer."""
    
    def __init__(self):
        super(FiniteStateMachine, self).__init__()
        self.outputs = []

    def output(self, input_value):
        """Return state's name as output."""
        return self.current_state.name

    def process(self, input_data, yield_none=True):
        """Process input data."""
        self.reset()
        for item in input_data:
            if yield_none: 
                yield self.output(item)
            elif self.output(item) is not None:
                yield self.output(item)
            self.transition(item)


class MooreMachine(Transducer):
    
    """Moore Machine."""

    def output(self, input_value):
        """Return output value assigned to the current state."""
        return self.current_state.output_values[0][1]


class MealyMachine(Transducer):
    
    """Mealy Machine."""

    def output(self, input_value):
        """Return output for a given state transition."""
        return dict(self.current_state.output_values).get(input_value)


if __name__ == "__main__":
    
    """
    fsm = FiniteStateMachine('TCP IP')
    
    closed = fsm.add_state("closed", initial=True)
    listen = fsm.add_state('LISTEN')
    synrcvd = fsm.add_state('SYN RCVD')
    established = fsm.add_state('ESTABLISHED')
    synsent = fsm.add_state('SYN SENT')
    finwait1 = fsm.add_state('FIN WAIT 1')
    finwait2 = fsm.add_state('FIN WAIT 2')
    timewait = fsm.add_state('TIME WAIT')
    closing = fsm.add_state('CLOSING')
    closewait = fsm.add_state('CLOSE WAIT')
    lastack = fsm.add_state('LAST ACK')
    
    fsm.add_transition(timewait, closed, '(wait)')
    fsm.add_transition(closed, listen, 'passive open')
    fsm.add_transition(closed, synsent, 'send SYN') 
    fsm.add_transition(synsent, closed, 'close timeout')
    fsm.add_transition(synsent, synrcvd, 'recv SYN, send SYN+ACK')
    fsm.add_transition(synsent, established, 'recv SYN+ACK, send ACK')
    fsm.add_transition(listen, synrcvd, 'recv SYN,send SYN+ACK')
    fsm.add_transition(listen, synsent, 'send SYN')

    synrcvd.update({'recv ACK': established, 'send FIN': finwait1,'recv RST': listen})
    established.update({'send FIN': finwait1, 'recv FIN, send ACK': closewait})
    closewait['send FIN'] = lastack
    lastack['recv ACK'] = closed
    finwait1.update({'send ACK': closing,'recv ACK': finwait2, r'recv FIN, ACK\n send ACK': timewait})
    finwait2[r'recv FIN,\nsend ACK'] = timewait
    closing[r'recv\nACK'] = timewait
    
    graph = fsm.graph()
    print graph
    filename = 'tcp.png'
    graph.draw(filename, prog='dot')
    from PIL import Image
    Image.open(filename).show()
    """
    
    fsm = FiniteStateMachine('sample fabrication process')
    
    job0 = fsm.add_state("digital model", initial=True)
    job1 = fsm.add_state("data generation")
    job2 = fsm.add_state("(robotic) pick and place")
    job3 = fsm.add_state("(robotic) scanning process")
    
    
    
    fsm.add_transition(None, job0, None, job1)
    fsm.add_transition('fabrication data', job1, 'send', job2)
    fsm.add_transition(None, job2, None, job3)
    fsm.add_transition('scanning data', job3, 'update', job0)
    
    
    fsm.show_graph()
    
    """
    fsm = FiniteStateMachine('sample fabrication process')
    
    job0 = fsm.add_state("digital model", initial=True)
    job1 = fsm.add_state("generate fabrication data")
    job2 = fsm.add_state("establish connection and initialize")
    job2 = fsm.add_state("send paths")
    job2 = fsm.add_state("get force")
    job2 = fsm.add_state("calculate new speed")
    job2 = fsm.add_state("send new speed")
    job3 = fsm.add_state("(robotic) scanning process")
    
    
    
    fsm.add_transition(job0, job1, 'done')
    fsm.add_transition(job1, job2, 'send')
    fsm.add_transition(job1, job3, 'send')
    fsm.add_transition(job2, job3, 'send')
    fsm.add_transition(job3, job0, 'update')
    
    job2.output_values = [("send", "robotic paths")]
    
    for key in job0:
        print key
    
    fsm.show_graph()
    """
    

    