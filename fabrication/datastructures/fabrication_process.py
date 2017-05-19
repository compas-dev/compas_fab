'''
Created on 15.05.2017

@author: rustr
'''

import networkx as nx
import matplotlib.pyplot as plt
from graphviz import Digraph # build upon compas graph structure ?
from fsm import FiniteStateMachine

import pygraphviz as pgv


class FabricationProcess(FiniteStateMachine):
    #===========================================================================
    """
    This class helps to plan, visualize and control the fabrication process. 
    A fabrication process consists of several jobs that can be performed one 
    after the other, or, run in parallel. The sequence of the process is defined 
    through the starting job and the transitions between the jobs (e.g. j1 and
    j2). Those transitions may have also a certain conditions implemented, which 
    is by default that j2 can be only performed, when j1 is done. A job can 
    consist of several tasks, that are specific to the fabrication process.
    For more info see https://en.wikipedia.org/wiki/Finite-state_machine
    """
        
    def add_job(self, name, initial=False):
        self.add_state(name, initial)
    
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
    
    """
    dot = Digraph(comment='The Round Table')
    dot.node('A', 'King Arthur')
    dot.node('B', 'Sir Bedevere the Wise')
    dot.node('L', 'Sir Lancelot the Brave')
    
    dot.engine = 'neato'
    
    
    
    dot.edges(['AB', 'AL'])
    dot.edge('B', 'L', constraint='false')
    
    print(dot.source)
    #dot.render('round-table.gv', view=True)
    #cmd = ['dot', '-Tpdf', '-O', "C:\\Users\\rustr\\workspace\\compas_fabrication\\fabrication\\datastructures\\test-output\\round-table.gv"]
    #proc = subprocess.Popen(cmd, startupinfo=STARTUPINFO)
    dot.render('test-output/round-table.gv', view=True)
    """

    G = pgv.AGraph()
    G.add_node('a')
    G.add_node('b')
    G.add_edge('a','b')
    
    G.node_attr['shape']='circle'
    G.add_node(1, color='red')
    print "here"
    G.layout(prog='dot')
    #G.layout()
    print "here"
    G.draw('file.png')
    print "here"
    