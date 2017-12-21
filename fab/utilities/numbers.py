import math

def map_range(value, from_min, from_max, to_min, to_max):
    """This function performs a linear interpolation of a value within the range
    of [from_min, from_max] to another range of [to_min, to_max].
    """
    from_range = from_max - from_min
    to_range = to_max - to_min
    value_scaled = (value - from_min)/float(from_range)
    return to_min + (value_scaled * to_range)

def arange(start, stop, step):
    """This function returns evenly spaced values within a given interval and is
    similar to NumPy's arange function.
    """
    if math.fabs(stop - (start + step)) > math.fabs(stop - start):
        raise ValueError("Please check the sign of step.")
    
    len = int(math.ceil((stop - start)/float(step)))
    return [start + i*step for i in range(len)]

    
    
    
if __name__ == "__main__":
    print(map_range(2, 0, 10, 0, 100))
    print(arange(3, -4, -0.2))
