import pytest

from compas_fab.robots import rfl


@pytest.fixture(autouse=True)
def add_imports(doctest_namespace):
    doctest_namespace["rfl"] = rfl
