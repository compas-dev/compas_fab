.. _install_compas_fab:

********************************************************************************
Install COMPAS FAB
********************************************************************************

.. highlight:: bash

**COMPAS FAB** can be easily installed on multiple platforms,
using popular package managers such as conda or pip.

Install with conda
==================

The recommended way to install **COMPAS FAB** is with `conda <https://conda.io/docs/>`_.
For example, create an environment named ``project_name`` and install COMPAS and COMPAS FAB.

::

    conda create -n project_name -c conda-forge compas_fab

Afterwards, simply activate the environment and run the following
command to check if the installation process was successful.

.. code-block:: bash

    conda activate project_name
    python -m compas_fab

.. code-block:: none

    Yay! COMPAS FAB is installed correctly!

You are ready to use **COMPAS FAB**!

Installation options
--------------------

Install COMPAS FAB in an environment with a specific version of Python.

.. code-block:: bash

    conda create -n project_name python=3.8 compas_fab

Install COMPAS FAB in an existing environment.

.. code-block:: bash

    conda install -n project_name compas_fab

Install with pip
================

Install COMPAS FAB using ``pip`` from the Python Package Index.

.. code-block:: bash

    pip install compas_fab

Install an editable version from local source.

.. code-block:: bash

    cd path/to/compas_fab
    pip install -e .

Note that installation with ``pip`` is also possible within a ``conda`` environment.

.. code-block:: bash

    conda activate project_name
    pip install -e .

.. note::

    On Windows, you may need to install
    `Microsoft Visual C++ 14.0 <https://www.scivision.dev/python-windows-visual-c-14-required/>`_.


Update with conda
=================

To update COMPAS FAB to the latest version with ``conda``

.. code-block:: bash

    conda update compas_fab

To switch to a specific version

.. code-block:: bash

    conda install compas_fab=1.0.2


Update with pip
===============

If you installed COMPAS FAB with ``pip`` the update command is the following

.. code-block:: bash

    pip install --upgrade compas_fab

Or to switch to a specific version

.. code-block:: bash

    pip install compas_fab==1.0.2


Next Steps
==========

* :ref:`Working with backends <backends>`
* :ref:`COMPAS FAB Examples <examples>`
* :ref:`COMPAS FAB API Reference <reference>`
* `COMPAS User Guide <https://compas.dev/compas/latest/userguide>`_
* `COMPAS API Reference <https://compas.dev/compas/latest/api>`_
