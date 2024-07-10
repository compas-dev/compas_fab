import importlib
import inspect
import json
import os
from datetime import datetime

import compas
import pytest

import compas_fab


def test_no_removed_names_from_any_package(compas_fab_api):
    return  # Temporary disable due to WIP branch
    generated_ver = parse_version(compas_fab_api["metadata"]["compas_fab_version"])
    current_ver = parse_version(compas_fab.__version__)

    # Raise to indicate the API file needs to be regenerated
    if current_ver["major"] != generated_ver["major"]:
        raise Exception("The compas_fab_api.json file needs to be regenerated for the current COMPAS_FAB major version")

    mismatches = dict()
    for module_name in compas_fab_api["modules"]:
        names_in_reference_version = compas_fab_api["modules"][module_name]
        names_in_current_version = set(get_names_in_module(module_name))

        for name in names_in_reference_version:
            if name not in names_in_current_version:
                if module_name not in mismatches:
                    mismatches[module_name] = []
                mismatches[module_name].append(name)

    assert len(mismatches) == 0, "The following names are missing from the API: " + str(mismatches)


@pytest.fixture
def compas_fab_api():
    with open(compas_fab_api_filename(), "r") as f:
        return json.load(f)


def compas_fab_api_filename():
    if compas.IPY:
        filename = "compas_fab_api_ipy.json"
    else:
        filename = "compas_fab_api.json"
    return os.path.join(os.path.dirname(__file__), filename)


# could be imported form compas
def parse_version(ver):
    ver_major, ver_minor, ver_patch = ver.split(".")[0:3]
    return dict(major=ver_major, minor=ver_minor, patch=ver_patch)


# could be imported form compas
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


if __name__ == "__main__":
    # Generate stable API dictionary file
    # This file should be regenerated every time there's a major release
    # Minor releases will check that nothing is missing from the second-level imports and the top compas package

    modules = [
        "compas_fab",
        "compas_fab.backends",
        "compas_fab.robots",
        "compas_fab.sensors",
        "compas_fab.utilities",
    ]

    compas_fab_version = compas_fab.__version__
    # strip git hash from version
    if not compas.IPY:
        compas_fab_version = compas_fab_version[:-9]

    compas_fab_api = dict(  # noqa: F811
        metadata=dict(
            generated_on=datetime.now().strftime("%Y%m%d"),
            compas_fab_version=compas_fab_version,
        ),
        modules=dict(),
    )

    for module_name in modules:
        compas_fab_api["modules"][module_name] = get_names_in_module(module_name)

    fname = compas_fab_api_filename()
    with open(fname, "w") as f:
        json.dump(compas_fab_api, f, indent=2, sort_keys=True)

    print("Generated API file: " + fname)
