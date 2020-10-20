from __future__ import print_function

import os
import sys

import pytest

HERE = os.path.dirname(__file__)
SRC = os.path.abspath(os.path.join(HERE, '..', 'src'))

if __name__ == '__main__':
    # Configure path to source
    if SRC not in sys.path:
        sys.path.append(SRC)

    # Fake Rhino modules
    pytest.load_fake_module('Rhino')
    pytest.load_fake_module('Rhino.Geometry', fake_types=['RTree', 'Sphere', 'Point3d'])

    pytest.run(HERE)
