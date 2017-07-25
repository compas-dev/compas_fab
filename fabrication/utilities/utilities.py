from __future__ import print_function
import sys

# TODO: maybe move this, as it is Rhino/Grasshopper specific
def unload_module(module_name):
    """Unload all modules that have module_name in its name.
    
    Example:
        unload_module('compas')
    """
    
    modules = filter(lambda m: m.find(module_name) != -1, sys.modules)

    for module in modules:
        sys.modules.pop(module)
        print("%s unloaded." % module)
        

def sign(number): 
    """Returns the sign of a number: +1 or -1.
    """
    return  int(int((number) > 0 ) - int((number) < 0 ))

def argsort(numbers):
    """Returns the indices that would sort a list of numbers.
    """
    numbers_copy = numbers[:]
    sorted_indices = []
    for i in range(len(numbers)):
        minv = min(numbers_copy)
        sorted_indices.append(numbers.index(minv))
        numbers_copy.remove(minv)
    return sorted_indices