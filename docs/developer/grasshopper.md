# Grasshopper components

The Grasshopper user objects are built with
[COMPAS Github Action componentizer](https://github.com/compas-dev/compas-actions.ghpython_components).

1. Apply your changes to the component source code under
   [`src/compas_fab/ghpython/components`](https://github.com/compas-dev/compas_fab/tree/main/src/compas_fab/ghpython/components).
2. Rebuild them:

    ```bash
    invoke build-ghuser-components --gh-io-folder=<path_to_ghio.dll>
    ```

3. Install them on Rhino/Grasshopper as usual:

    ```bash
    python -m compas_rhino.install
    ```

## CPython components (Rhino 8+)

For the CPython Grasshopper variants used in Rhino 8 and later:

```bash
invoke build-cpython-ghuser-components
```

The CPython component sources live under
[`src/compas_fab/ghpython/components_cpython`](https://github.com/compas-dev/compas_fab/tree/main/src/compas_fab/ghpython/components_cpython).
