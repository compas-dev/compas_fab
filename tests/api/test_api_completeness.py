from __future__ import absolute_import

import os
import pytest
import shutil
import subprocess
import importlib
import inspect
import compas


def get_names_in_module(module_name):
    exceptions = ["absolute_import", "division", "print_function"]
    module = importlib.import_module(module_name)
    all_names = module.__all__ if hasattr(module, "__all__") else dir(module)
    return sorted(
        [
            i
            for i in all_names
            if not i.startswith("_") and i not in exceptions and not inspect.ismodule(getattr(module, i))
        ]
    )


@pytest.fixture
def compas_fab_api():
    if compas.IPY:
        return

    modules = [
        "compas_fab.backends",
        "compas_fab.robots",
        "compas_fab.sensors",
        "compas_fab.utilities",
    ]
    api = {}
    for module_name in modules:
        api[module_name] = get_names_in_module(module_name)
    return api


@pytest.fixture
def compas_fab_stubs():
    if compas.IPY:
        return

    env = compas._os.prepare_environment()

    HERE = os.path.dirname(__file__)
    HOME = os.path.abspath(os.path.join(HERE, "../.."))
    TEMP = os.path.abspath(os.path.join(HOME, "temp/stubs"))
    DOCS = os.path.abspath(os.path.join(HOME, "docs"))
    API = os.path.abspath(os.path.join(DOCS, "api"))

    shutil.rmtree(TEMP, ignore_errors=True)

    _, _, filenames = next(os.walk(API))
    for name in filenames:
        if name == "compas_fab.rst" or not name.startswith("compas_fab."):
            continue
        stub = os.path.abspath(os.path.join(API, name))
        subprocess.call("sphinx-autogen -o {} {}".format(TEMP, stub), shell=True, env=env)

    _, _, filenames = next(os.walk(TEMP))

    shutil.rmtree(TEMP, ignore_errors=True)

    stubs = {}
    for name in filenames:
        parts = name.split(".")
        if len(parts) != 4:
            continue
        package = parts[0]
        module = parts[1]
        item = parts[2]
        if package == "compas_fab":
            packmod = "{}.{}".format(package, module)
            if packmod not in stubs:
                stubs[packmod] = []
            stubs[packmod].append(item)

    return stubs


def test_compas_api_stubs(compas_fab_api, compas_fab_stubs):
    if compas.IPY:
        return

    for packmod in compas_fab_api:
        parts = packmod.split(".")
        if len(parts) != 2:
            continue
        assert packmod in compas_fab_stubs
        for name in compas_fab_api[packmod]:
            if name in ["Configuration", "PosCon3D", "PosConCM"]:
                continue
            # deprecated functions
            if name in ["read_data_from_json", "write_data_to_json"]:
                continue
            assert name in compas_fab_stubs[packmod], "missing {} in {}".format(name, packmod)
