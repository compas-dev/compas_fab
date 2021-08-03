********************************************************************************
Getting started
********************************************************************************

.. highlight:: bash

**COMPAS FAB** can be easily installed on multiple platforms,
using popular package managers such as conda or pip.

Install with conda
==================

The recommended way to install **COMPAS FAB** is with `conda <https://conda.io/docs/>`_.
For example, create an environment named ``my-project`` and install COMPAS and COMPAS FAB.

::

    conda config --add channels conda-forge
    conda create -n my-project compas_fab

Afterwards, simply activate the environment
and run the following command to check if the installation process was successful.

.. code-block:: bash

    conda activate my-project
    python -c "import compas_fab; print(compas_fab.__version__)"

.. code-block:: none

    0.19.1

You are ready to use **COMPAS FAB**!

Installation options
--------------------

Install COMPAS FAB in an environment with a specific version of Python.

.. code-block:: bash

    conda create -n my-project python=3.8 compas_fab

Install COMPAS FAB in an existing environment.

.. code-block:: bash

    conda install -n my-project compas_fab

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

    conda activate my-project
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

    conda install compas_fab=0.19.1


Update with pip
===============

If you installed COMPAS FAB with ``pip`` the update command is the following

.. code-block:: bash

    pip install --upgrade compas_fab

Or to switch to a specific version

.. code-block:: bash

    pip install compas_fab==0.19.1


Working in Rhino
================

To make **COMPAS FAB** available inside Rhino, open the *command prompt*,
activate the appropriate environment, and type the following:

::

    python -m compas_rhino.install

.. note:

    On Windows, you might need to run the *command prompt* as administrator
    before running the install command.

Open Rhino, start the Python script editor, type ``import compas_fab`` and
run it to verify that your installation is working.

Making **COMPAS FAB** available in Rhino also installs a suite of Grasshopper
components with **COMPAS FAB** functionality.  See
:ref:`ROS in Grasshopper <examples_ros_in_grasshopper>` for an example.

Working in Blender
==================

Once **COMPAS** itself is installed for Blender following the
`documented procedure <https://compas.dev/compas/latest/gettingstarted/blender.html>`_,
**COMPAS FAB** will automatically be available as well after installing it.


Working in Visual Studio Code
=============================

`Visual Studio Code <https://code.visualstudio.com/>`_ is a free and open source text
editor with very good support for Python programming.

We recommend installing the following VS Code extensions:

* `Python <https://marketplace.visualstudio.com/items?itemName=ms-python.python>`_

  *Official extension to add support for Python programming, including
  debugging, auto-complete, formatting, etc.*

* `Docker <https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker>`_

  *Add support for ``Dockerfile`` and ``docker-compose.yml`` files to VS Code.*

* `EditorConfig <https://marketplace.visualstudio.com/items?itemName=EditorConfig.EditorConfig>`_

  *Add support for ``.editorconfig`` files to VS Code.*

To install the above extensions, open the ``Extensions`` view  by clicking on
the corresponding icon in the **Activity Bar** on the left side of VS Code
and search the extension name in the search box. Once found, select it and
click ``Install``.

We recommend tweaking some of the default VS Code settings:

* Python Linter:

  Select ``flake8`` as your default python linter: open the ``Command Palette``
  (``Ctrl+Shift+P``), type ``Python: Select Linter``, select it and select
  ``flake8`` from the list.

* *[Windows Only]* Default Shell:

  Change the default shell from ``PowerShell`` to ``Command Prompt``: open the
  ``Command Palette`` (``Ctrl+Shift+P``), type ``Select Default Shell``,
  select it and from the options, select ``Command Prompt``.
  Kill all opened terminals for it to take effect.

Run scripts
-----------

To run Python scripts from within VS Code, simply open the file and press
``F5``. This will start the script with the debugger attached, which means
you can add breakpoints (clicking on the gutter, next to the line numbers),
inspect variables and step into your code for debugging.

Alternatively, use ``Ctrl+F5`` to start the script without debugger.

Virtual environments
--------------------

If you are using ``conda`` to manage your virtual environments, VS Code has
built-in support for them. When a ``.py`` file is open on VS Code, the bottom
left side of the **Status bar** will show the Python interpreter used to run
scripts. Click on it and a list of all available interpreters including all
environments will be shown. Select one, and the next time you run a script,
the newly selected interpreter will be used.


Next Steps
==========

* :ref:`Working with backends <backends>`
* :ref:`COMPAS FAB Examples <examples>`
* :ref:`COMPAS FAB API Reference <reference>`
* `COMPAS Tutorials <https://compas.dev/compas/latest/tutorial.html>`_
* `COMPAS API Reference <https://compas.dev/compas/latest/api.html>`_
