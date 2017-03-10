# compAS:Fabrication package

Site package to the compAS Framework aimed at providing tools for robotic fabrication.

## Getting started

Just as the compAS Framework, the suggested usage is not through an install, but rather a local
copy of the source code.

Start off by making sure you have all the prerequisites installed. Open a terminal window on the
folder where you cloned this repository and run the following:

    pip install -r requirements.txt

And add the root folder of this repository to your `PYTHONPATH` to make it importable.

Fire off your favorite editor and run the simplest example:

```python
from compas_fabrication.fabrication.robots.rfl import Simulator

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
