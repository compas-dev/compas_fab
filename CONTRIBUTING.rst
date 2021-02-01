.. _contributors_guide:

Contributor's Guide
===================

Contributions are always welcome and greatly appreciated!

Code contributions
------------------

We love pull requests from everyone! Here's a quick guide to improve the code:

1. Fork `the repository <https://github.com/compas-dev/compas_fab>`_ and clone the fork.
2. Create a virtual environment using your tool of choice (e.g. ``virtualenv``, ``conda``, etc).
3. Install development dependencies:

::

    pip install -r requirements-dev.txt

4. Make sure all tests pass:

::

    invoke test

5. Start making your changes to the **master** branch (or branch off of it).
6. Make sure all tests still pass:

::

    invoke test

7. Check there are no linter errors:

::

    invoke lint

8. Add yourself to ``AUTHORS.rst``.
9. Commit your changes and push your branch to GitHub.
10. Create a `pull request <https://help.github.com/articles/about-pull-requests/>`_ through the GitHub website.


During development, use `pyinvoke <http://docs.pyinvoke.org/>`_ tasks on the
command prompt to ease recurring operations:

* ``invoke clean``: Clean all generated artifacts.
* ``invoke check``: Run various code and documentation style checks.
* ``invoke docs``: Generate documentation.
* ``invoke lint``: Run code linter for coding style checks.
* ``invoke test``: Run all tests and checks in one swift command.
* ``invoke``: Show available tasks.


Documentation improvements
--------------------------

We could always use more documentation, whether as part of the
introduction/examples/usage documentation or API documentation in docstrings.

Documentation is written in `reStructuredText <http://docutils.sourceforge.net/rst.html>`_
and use `Sphinx <http://sphinx-doc.org/index.html>`_ with
`numpy-style docstrings <https://sphinxcontrib-napoleon.readthedocs.io/en/latest/example_numpy.html>`_
to generate the HTML output.

Once you made the documentation changes locally, run the documentation generation::

    invoke docs


Bug reports
-----------

When `reporting a bug <https://github.com/compas-dev/compas_fab/issues>`_
please include:

* Operating system name and version.
* Python version.
* Any details about your local setup that might be helpful in troubleshooting.
* Detailed steps to reproduce the bug.

Feature requests and feedback
-----------------------------

The best way to send feedback is to file an issue on
`Github <https://github.com/compas-dev/compas_fab/issues>`_. If you are proposing a feature:

* Explain in detail how it would work.
* Keep the scope as narrow as possible, to make it easier to implement.

A Note on Architecture for Backend Clients
------------------------------------------

To maintain consistency from one backend client to another and to promote modularity,
we make use of several interfaces.  Please reference :ref:`Note on Architecture <architecture>`
for more details on how to add or amend a backend client.

Design documents
----------------

.. toctree::
    :maxdepth: 1

    architecture