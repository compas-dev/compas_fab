.. _setup_frontend_vscode:

*******************************************************************************
Setup Visual Studio Code
*******************************************************************************

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
right side of the **Status bar** will show the Python interpreter used to run
scripts. Click on it and a list of all available interpreters including all
environments will be shown. Select one, and the next time you run a script,
the newly selected interpreter will be used.
