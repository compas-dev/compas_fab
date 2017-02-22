# compAS:Robot package

Side package to the compAS Framework aimed a providing tools for robotic fabrication.

## Getting started

Just as the compAS Framework, the suggested usage is not through an install, but rather a local
copy of the source.

Start off by making sure you have all the prerequisites installed:

    $ pip install -r requirements.txt

And add the root folder of this repository to your `PYTHONPATH` to make it importable.

Fire off your favorite editor and run the simplest example:

```python
from compas_robot.rfl import Simulator

with Simulator() as simulator:
    print ('Connected: ' + str(simulator.is_connected()))

print ('Done')

```


## Build

All local development tasks are handled by `pyinvoke` through the `inv` (aka `invoke`) command.

To build the documentation locally:

    $ inv build

To run all docstring tests/examples:

    $ inv doctest
