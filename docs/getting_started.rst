********************************************************************************
Getting started
********************************************************************************

.. highlight:: bash

Installation
============

The recommended way to install **COMPAS FAB** is to use `Anaconda/conda <https://conda.io/docs/>`_:

::

    conda install -c conda-forge compas_fab


But it can also be installed using ``pip``:

::

    pip install compas_fab


.. note::

    On Windows, you may need to install
    `Microsoft Visual C++ 14.0 <https://www.scivision.co/python-windows-visual-c++-14-required/>`_.


Once the installation is completed, you can verify your setup.
Start Python from the command prompt and run the following:

::

    >>> import compas_fab

Now you can make **COMPAS FAB** available inside your favorite CAD software, or use it from your text editor
or IDE of choice.


Working in Rhino
================

Installing **COMPAS FAB** for Rhino is very simple. Just open the *command prompt*
and type the following:

::

    python -m compas_fab.rhino.install -v 6.0


To install on Rhinoceros 5.0 instead, use ``-v 5.0``.

.. note:

    On Windows, you might need to run the *command prompt* as administrator before running
    the install command.


Working in Blender
==================

Once **COMPAS** itself is installed for Blender following the
`documented procedure <https://compas-dev.github.io/main/environments/blender.html>`_,
**COMPAS FAB** will automatically be available as well after installing it.


Working in Visual Studio Code
=============================

`Visual Studio Code <https://code.visualstudio.com/>`_ is a free and open source text
editor with very good support for Python programming.

We recommend installing the following VS Code extensions:

* `Python <https://marketplace.visualstudio.com/items?itemName=ms-python.python>`_

  *Official extension to add support for Python programming, including debugging, auto-complete, formatting, etc.*

* `EditorConfig <https://marketplace.visualstudio.com/items?itemName=EditorConfig.EditorConfig>`_

  *Add support for ``.editorconfig`` files to VS Code.*

To install the above extensions, open the ``Extensions`` view  by clicking on the
``Extensions`` icon in the **Activity Bar** on the left side of VS Code and search
the extension name in the search box. Once found, select it and click ``Install``.

By default, VS Code will use ``PyLint`` to verify your code. To select a different
linter: open the ``Command Palette`` (``Ctrl+Shift+P``) and select the
``Python: Select Linter`` command.

Run scripts
-----------

To run Python scripts from within VS Code, simply open the file and press ``F5``.
This will start the script with the debugger attached, which means you can add
breakpoints (clicking on the gutter, next to the line numbers), inspect variables
and step into your code for debugging.

Alternatively, use ``Ctrl+F5`` to start the script without debugger.

Virtual environments
--------------------

If you are using ``conda`` to manage your virtual environments, VS Code has built-in
support for them. When a ``.py`` file is open on VS Code, the bottom left side of the
**Status bar** will show the Python interpreter used to run scripts.
Click on it and a list of all available interpreters including all environments
will be shown. Select one, and the next time you run a script, the newly selected
interpreter will be used.


First Steps
===========

* :ref:`Working with backends <backends>`
* :ref:`COMPAS FAB Examples <examples>`
* :ref:`COMPAS FAB API Reference <reference>`
* `COMPAS Examples <https://compas-dev.github.io/main/examples.html>`_
* `COMPAS Tutorials <https://compas-dev.github.io/main/tutorial.html>`_
* `COMPAS API Reference <https://compas-dev.github.io/main/api.html>`_
