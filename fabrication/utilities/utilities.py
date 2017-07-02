'''
Created on 19.06.2017

@author: rustr
'''
import sys

def unload_module(module_name):
    """ Reload all modules that have module_name in its name.
        e.g. reload_module('compas')
    """
    
    modules_to_pop = []
    for mod in sys.modules:
        if mod.find(module_name) != -1:
            modules_to_pop.append(mod)
    
    for mod in modules_to_pop:
        if mod in sys.modules:
            sys.modules.pop(mod)
            print "%s unloaded." % mod