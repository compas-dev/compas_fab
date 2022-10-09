from __future__ import print_function

import os

from invoke import task
from invoke import Collection

from compas_invocations.console import chdir
from compas_invocations import build
from compas_invocations import docs
from compas_invocations import style
from compas_invocations import tests

BASE_FOLDER = os.path.dirname(__file__)


@task
def lint(ctx):
    """Check the consistency of coding style."""
    print("Running flake8 python linter...")

    with chdir(BASE_FOLDER):
        ctx.run("flake8 src")


ns = Collection(
    docs.help,
    style.check,
    lint,  # until we switch to black, we keep this local flake8-based lint task
    # style.lint,
    style.format,
    docs.docs,
    docs.linkcheck,
    tests.test,
    tests.testdocs,
    tests.testcodeblocks,
    build.prepare_changelog,
    build.clean,
    build.release,
    build.build_ghuser_components,
)
ns.configure(
    {
        "base_folder": BASE_FOLDER,
        "ghuser": {
            "source_dir": "src/compas_fab/ghpython/components",
            "target_dir": "src/compas_fab/ghpython/components/ghuser",
            "prefix": "(COMPAS FAB)",
        },
    }
)
