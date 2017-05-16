'''
Created on 15.05.2017

@author: rustr
'''

import networkx as nx
import matplotlib.pyplot as plt
from graphviz import Digraph

import pygraphviz as pgv


class FabricationProcess(object):
    """
    This class helps to plan the fabrication process and . 
    A fabrication process consists of several jobs, that can be performed one after the other,
    or in parallel. The order is defined in the transition between two jobs (e.g. job01 and job02), 
    that is based on a certain condition. The default condition is that job02 can be only performed,
    when job01 is done.
    A job can consist of several tasks, 
     conditions define 
    
    For more info see https://en.wikipedia.org/wiki/Finite-state_machine
    """
    def __init__(self):
        pass
    
    def add_job(self, name):
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
    