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

You are ready to use **COMPAS FAB**!

Update
======

Update with conda
-----------------

To update COMPAS to the latest version with ``conda``

.. code-block:: bash

    conda update compas_fab

To switch to a specific version

.. code-block:: bash

    conda install compas_fab=0.15.0


Update with pip
---------------

If you installed COMPAS with ``pip`` the update command is the following

.. code-block:: bash

    pip install --upgrade compas_fab

Or to switch to a specific version

.. code-block:: bash

    pip install compas_fab==0.15.0


Working in Rhino
================

To make **COMPAS FAB** available inside Rhino, open the *command prompt*
and type the following:

::

    python -m compas_rhino.install

.. note:

    On Windows, you might need to run the *command prompt* as administrator
    before running the install command.

Open Rhino, start the Python script editor, type ``import compas_fab`` and
run it to verify that your installation is working.

Working in Blender
==================

Once **COMPAS** itself is installed for Blender following the
`documented procedure <https://compas.dev/compas/gettingstarted/cad/blender.html>`_,
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
* `COMPAS Tutorials <https://compas.dev/compas/tutorial.html>`_
* `COMPAS API Reference <https://compas.dev/compas/api.html>`_
