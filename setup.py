#!/usr/bin/env python
# -*- encoding: utf-8 -*-
from __future__ import absolute_import
from __future__ import print_function

import io
from glob import glob
from os import path

from setuptools import find_packages
from setuptools import setup

here = path.abspath(path.dirname(__file__))


def read(*names, **kwargs):
    return io.open(path.join(here, *names), encoding=kwargs.get("encoding", "utf8")).read()


about = {}
exec(read("src", "compas_fab", "__version__.py"), about)

long_description = read("README.md")
requirements = read("requirements.txt").split("\n")
optional_requirements = {}

setup(
    name=about["__title__"],
    version=about["__version__"],
    license=about["__license__"],
    description=about["__description__"],
    author=about["__author__"],
    author_email=about["__author_email__"],
    url=about["__url__"],
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=find_packages("src"),
    package_dir={"": "src"},
    py_modules=[path.splitext(path.basename(path))[0] for path in glob("src/*.py")],
    include_package_data=True,
    zip_safe=False,
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Operating System :: Unix",
        "Operating System :: POSIX",
        "Operating System :: Microsoft :: Windows",
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: Implementation :: CPython",
        "Programming Language :: Python :: Implementation :: IronPython",
        "Topic :: Scientific/Engineering",
    ],
    keywords=["robotic fabrication", "digital fabrication", "architecture", "robotics", "ros"],
    install_requires=requirements,
    extras_require={},
    entry_points={},
)
