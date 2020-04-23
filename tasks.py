# -*- coding: utf-8 -*-
from __future__ import print_function

import codecs
import contextlib
import glob
import os
import sys
from shutil import copytree
from shutil import rmtree
from xml.dom.minidom import parse

from invoke import Exit
from invoke import task

try:
    input = raw_input
except NameError:
    pass
BASE_FOLDER = os.path.dirname(__file__)


class Log(object):
    def __init__(self, out=sys.stdout, err=sys.stderr):
        self.out = out
        self.err = err

    def flush(self):
        self.out.flush()
        self.err.flush()

    def write(self, message):
        self.flush()
        self.out.write(message + '\n')
        self.out.flush()

    def info(self, message):
        self.write('[INFO] %s' % message)

    def warn(self, message):
        self.write('[WARN] %s' % message)


log = Log()


def confirm(question):
    while True:
        response = input(question).lower().strip()

        if not response or response in ('n', 'no'):
            return False

        if response in ('y', 'yes'):
            return True

        print('Focus, kid! It is either (y)es or (n)o', file=sys.stderr)


@task(default=True)
def help(ctx):
    """Lists available tasks and usage."""
    ctx.run('invoke --list')
    log.write('Use "invoke -h <taskname>" to get detailed help for a task.')


@task(help={
    'docs': 'True to generate documentation, otherwise False',
    'bytecode': 'True to clean up compiled python files, otherwise False.',
    'builds': 'True to clean up build/packaging artifacts, otherwise False.'})
def clean(ctx, docs=True, bytecode=True, builds=True):
    """Cleans the local copy from compiled artifacts."""
    if builds:
        ctx.run('python setup.py clean')

    if bytecode:
        for root, dirs, files in os.walk(BASE_FOLDER):
            for f in files:
                if f.endswith('.pyc'):
                    os.remove(os.path.join(root, f))
            if '.git' in dirs:
                dirs.remove('.git')

    folders = []

    if docs:
        folders.append('docs/_build/')
        folders.append('docs/reference/generated')
        folders.append('dist/')

    if bytecode:
        folders.append('src/compas_fab/__pycache__')

    if builds:
        folders.append('build/')
        folders.append('src/compas_fab.egg-info/')

    for folder in folders:
        rmtree(os.path.join(BASE_FOLDER, folder), ignore_errors=True)


@task(help={
      'rebuild': 'True to clean all previously built docs before starting, otherwise False.',
      'doctest': 'True to run doctests, otherwise False.',
      'check_links': 'True to check all web links in docs for validity, otherwise False.'})
def docs(ctx, doctest=False, rebuild=False, check_links=False):
    """Builds package's HTML documentation."""

    if rebuild:
        clean(ctx)

    with chdir(BASE_FOLDER):
        if doctest:
            ctx.run('sphinx-build -b doctest docs dist/docs')

        ctx.run('sphinx-build -b html docs dist/docs')

        if check_links:
            ctx.run('sphinx-build -b linkcheck docs dist/docs')


@task()
def lint(ctx):
    """Check the consistency of coding style."""
    log.write('Running flake8 python linter...')
    ctx.run('flake8 src')


@task()
def check(ctx):
    """Check the consistency of documentation, coding style and a few other things."""

    with chdir(BASE_FOLDER):
        lint(ctx)

        log.write('Checking MANIFEST.in...')
        ctx.run('check-manifest --ignore-bad-ideas=remoteApi.so')

        log.write('Checking ReStructuredText formatting...')
        ctx.run('python setup.py check --strict --metadata --restructuredtext')

        # log.write('Checking python imports...')
        # ctx.run('isort --check-only --diff --recursive src tests setup.py')


@task()
def deploy_docs(ctx):
    """Deploy docs."""
    temp_folder = os.path.join(BASE_FOLDER, 'temp')
    docs_folder = os.path.join(temp_folder, 'docs')
    log.write('Cleaning up temp docs folder %s' % docs_folder)
    rmtree(docs_folder, ignore_errors=True)

    log.write('Cloning github repository %s/docs' % temp_folder)
    with chdir(temp_folder):
        ctx.run('git clone https://github.com/gramaziokohler/gramaziokohler.github.io.git docs')

    target_doc_folder = os.path.join(docs_folder, 'compas_fab', 'latest')
    log.write('Removing old docs from folder %s' % target_doc_folder)
    rmtree(target_doc_folder, ignore_errors=True)

    log.write('Copy current docs')
    copytree(os.path.join(BASE_FOLDER, 'dist', 'docs'), target_doc_folder)

    with chdir(target_doc_folder):
        ctx.run('git add . && git commit -m doc-deployer && git push')


@task(help={
      'checks': 'True to run all checks before testing, otherwise False.',
      'doctest': 'True to run doctest modules, otherwise False.',
      'coverage': 'True to generate coverage report using pytest-cov'
      })
def test(ctx, checks=False, doctest=False, coverage=False):
    """Run all tests."""
    if checks:
        check(ctx)

    with chdir(BASE_FOLDER):
        pytest_args = ['pytest']
        if doctest:
            pytest_args.append('--doctest-modules')
        if coverage:
            pytest_args.append('--cov=compas_fab')

        ctx.run(" ".join(pytest_args))


@task
def prepare_changelog(ctx):
    """Prepare changelog for next release."""
    UNRELEASED_CHANGELOG_TEMPLATE = '\nUnreleased\n----------\n\n**Added**\n\n**Changed**\n\n**Removed**\n'

    with chdir(BASE_FOLDER):
        # Preparing changelog for next release
        with open('CHANGELOG.rst', 'r+') as changelog:
            content = changelog.read()
            start_index = content.index('----------')
            start_index = content.rindex('\n', 0, start_index - 1)
            last_version = content[start_index:start_index + 11].strip()

            if last_version == 'Unreleased':
                log.write('Already up-to-date')
                return

            changelog.seek(0)
            changelog.write(content[0:start_index] + UNRELEASED_CHANGELOG_TEMPLATE + content[start_index:])

        ctx.run('git add CHANGELOG.md && git commit -m "Prepare changelog for next release"')


@task(help={
      'release_type': 'Type of release follows semver rules. Must be one of: major, minor, patch.'})
def release(ctx, release_type):
    """Releases the project in one swift command!"""
    if release_type not in ('patch', 'minor', 'major'):
        raise Exit('The release type parameter is invalid.\nMust be one of: major, minor, patch')

    # Run checks
    ctx.run('invoke check test')

    # Bump version and git tag it
    ctx.run('bump2version %s --verbose' % release_type)

    # Build project
    ctx.run('python setup.py clean --all sdist bdist_wheel')

    # Upload to pypi
    if confirm('You are about to upload the release to pypi.org. Are you sure? [y/N]'):
        files = ['dist/*.whl', 'dist/*.gz', 'dist/*.zip']
        dist_files = ' '.join([pattern for f in files for pattern in glob.glob(f)])

        if len(dist_files):
            ctx.run('twine upload --skip-existing %s' % dist_files)
        else:
            raise Exit('No files found to release')
    else:
        raise Exit('Aborted release')


@contextlib.contextmanager
def chdir(dirname=None):
    current_dir = os.getcwd()
    try:
        if dirname is not None:
            os.chdir(dirname)
        yield
    finally:
        os.chdir(current_dir)
