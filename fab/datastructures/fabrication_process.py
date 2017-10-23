'''
Created on 15.05.2017

@author: rustr
'''

import matplotlib.pyplot as plt
from graphviz import Digraph # build upon compas graph structure ?
from fsm import FiniteStateMachine

class FabricationProcess(FiniteStateMachine):
    #===========================================================================
    """
    This class helps to plan, visualize and control the fabrication process. 
    A fabrication process consists of several jobs that can be performed one 
    after the other, or, run in parallel. The sequence of the process is defined 
    through the starting job and the transitions between the jobs (e.g. job1 and
    job2). Those transitions may have also a certain conditions implemented, which 
    is by default that job2 can be only performed, when job1 is done. A job can 
    consist of several tasks, that are specific to the fabrication process.
    For more info see https://en.wikipedia.org/wiki/Finite-state_machine
    """
        
    def add_job(self, name, initial=False):
        return self.add_state(name, initial)
    
    def next_job(self):
        pass
    
    def pause(self):
        pass
    
    def resume(self):
        pass
    
    def stop(self):
        pass
    
    def start(self):
        pass
    
    def run(self):
        pass
        
    

if __name__ == "__main__":
    
    f = FabricationProcess('fabrication process')
    
    job0 = f.add_job("digital model", initial=True)
    job1 = f.add_job("data generation")
    job2 = f.add_job("(robotic) pick and place")
    job3 = f.add_job("(robotic) scanning process")
    
    f.add_transition(job0, job1, input_value=None, action='start')
    f.add_transition(job1, job2, input_value='fabrication data', action='send')
    f.add_transition(job2, job3)
    f.add_transition(job3, job0, input_value='scanning data', action='update')
        
    
    f.next(None)
    f.next('fabrication data')    
    f.next(None)
    f.next("scanning data")
    
    f.show()
    
    """
    f = FabricationProcess('sample fabrication process')
    
    job0 = fp.add_state("digital model", initial=True)
    job1 = fp.add_state("data generation")
    job2 = fp.add_state("send robotic paths")
    job2 = fp.add_state("get sensor data")
    job2 = fp.add_state("calculate and send new speed")
    job3 = fp.add_state("(robotic) scanning process")
    
    fsm.add_transition(job0, job1, action='start')
    fsm.add_transition(job1, job2, 'send')
    fsm.add_transition(job1, job3, 'send')
    fsm.add_transition(job2, job3, 'send')
    fsm.add_transition(job3, job0, 'update')
    
    job2.output_values = [("send", "robotic paths")]
    
    for key in job0:
        print key
    
    fsm.show_graph()
    """
    
    
