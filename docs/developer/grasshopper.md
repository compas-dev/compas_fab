# Grasshopper components

The Grasshopper user objects are built with
[COMPAS Github Action componentizer](https://github.com/compas-dev/compas-actions.ghpython_components).

1. Apply your changes to the component source code under
   [`src/compas_fab/ghpython/components_cpython`](https://github.com/compas-dev/compas_fab/tree/main/src/compas_fab/ghpython/components_cpython).
2. Rebuild them:

    ```bash
    invoke build-cpython-ghuser-components
    ```

3. Install them on Rhino/Grasshopper simply by dropping the generated files into Grasshopper.
