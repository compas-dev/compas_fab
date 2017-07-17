# pylint: disable=C0103
import contextlib
import os
import sys
from invoke import Collection, task


@task(help={
    'docs': 'True to generate documentation, otherwise False',
    'bytecode': 'Tue to generate compiled python files, otherwise False.'})
def clean(ctx, docs=True, bytecode=False):
    """Cleans the local copy from compiled artifacts."""
    patterns = []
    if docs:
        patterns.append('_docs/_build')
        patterns.append('_docs/source/pages/reference/generated')
    if bytecode:
        patterns.append('**/*.pyc')
    for pattern in patterns:
        ctx.run('rm -rf %s' % pattern)


@task(help={
    'docs': 'True to clean generated documentation, otherwise False.'})
def build(ctx, docs=True):
    """Builds the current package."""
    if docs:
        with chdir('_docs/'):
            ctx.run('python make.py && sphinx-build -M html source _build')


@task()
def doctest(ctx):
    """Runs all examples defined in docstrings for this module."""
    ctx.run('pytest --doctest-module')


@contextlib.contextmanager
def chdir(dirname=None):
    current_dir = os.getcwd()
    try:
        if dirname is not None:
            os.chdir(dirname)
        yield
    finally:
        os.chdir(current_dir)


namespace = Collection()
namespace.add_task(clean)
namespace.add_task(build)
namespace.add_task(doctest)

# Workaround for Windows execution
if sys.platform == 'win32':
    win32_config = {'run': {'shell': os.environ['COMSPEC']}}
    namespace.configure(win32_config)
