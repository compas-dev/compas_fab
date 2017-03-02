# pylint: disable=C0103
import contextlib
import os
import sys
from invoke import Collection, task


@task(help={
    'docs': 'Indicates whether to clean generated documentation or not.',
    'bytecode': 'Indicates whether to clean compiled python files or not.'})
def clean(ctx, docs=True, bytecode=False):
    patterns = []
    if docs:
        patterns.append('docs/_build')
    if bytecode:
        patterns.append('**/*.pyc')
    for pattern in patterns:
        ctx.run("rm -rf %s" % pattern)


@task(help={
    'docs': 'Indicates whether to clean generated documentation or not.'})
def build(ctx, docs=True):
    if docs:
        with chdir('docs/'):
            ctx.run("python make.py && make html")


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

# Workaround for Windows execution
if sys.platform == 'win32':
    win32_config = {'run': {'shell': os.environ['COMSPEC']}}
    namespace.configure(win32_config)
