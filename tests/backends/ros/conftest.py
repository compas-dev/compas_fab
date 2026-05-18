"""Shared fixtures for tests that talk to a live ROS stack over rosbridge.

Two separate fixtures, one per stack — each skips independently when its
stack isn't reachable, so a test file can opt into either or both:

    def test_something(ros1_client):  # only runs against the ROS 1 stack
        ...

    def test_other(ros2_client):  # only runs against the ROS 2 stack
        ...

The defaults match the host-port defaults baked into the corresponding
docker-compose files (see ``tests/integration_setup/README.md``):

| Var                       | Default | Stack |
|---------------------------|---------|-------|
| COMPAS_FAB_ROS_HOST       | localhost | both |
| COMPAS_FAB_ROS1_PORT      | 9090    | ROS 1 |
| COMPAS_FAB_ROS2_PORT      | 9091    | ROS 2 |

Both fixtures require ``COMPAS_FAB_RUN_ROS_INTEGRATION_TESTS=1`` to opt in;
otherwise every test that uses them is skipped.
"""
import os

import pytest

# NOTE: ``RosClient`` (and the rest of ``compas_fab.backends.ros``) is
# imported lazily inside ``_make_ros_client``. The top-level ``conftest.py``
# installs Twisted's ``selectreactor`` in ``pytest_configure``, but pytest
# imports every ``conftest.py`` in the tree *before* dispatching the
# ``pytest_configure`` hooks. Importing ``RosClient`` at module load pulls
# in ``roslibpy`` → ``twisted.internet.reactor``, which installs the default
# reactor; the subsequent ``selectreactor.install()`` then raises
# ``ReactorAlreadyInstalledError``.

RUN_ENV_VAR = "COMPAS_FAB_RUN_ROS_INTEGRATION_TESTS"
HOST_ENV_VAR = "COMPAS_FAB_ROS_HOST"
ROS1_PORT_ENV_VAR = "COMPAS_FAB_ROS1_PORT"
ROS2_PORT_ENV_VAR = "COMPAS_FAB_ROS2_PORT"

# Backwards-compatible alias for the older single-port var. If set, takes
# precedence over `COMPAS_FAB_ROS1_PORT` for the ROS 1 fixture (assumes the
# only stack the user was previously testing was on the legacy port).
LEGACY_PORT_ENV_VAR = "COMPAS_FAB_ROS_PORT"

ROS1_DEFAULT_PORT = 9090
ROS2_DEFAULT_PORT = 9091


def _skip_if_not_opted_in():
    if os.environ.get(RUN_ENV_VAR) != "1":
        pytest.skip(
            "ROS integration tests are opt-in. Set {0}=1 and start at least one stack:\n"
            "  ROS 1: docker compose -f tests/integration_setup/docker-compose.yml up -d\n"
            "         (rosbridge on {1}, default {2})\n"
            "  ROS 2: docker compose -f docs/installation/docker_files/ros2-ur10e-demo/docker-compose.yml up -d\n"
            "         (rosbridge on {3}, default {4})\n"
            "See tests/integration_setup/README.md.".format(
                RUN_ENV_VAR,
                ROS1_PORT_ENV_VAR,
                ROS1_DEFAULT_PORT,
                ROS2_PORT_ENV_VAR,
                ROS2_DEFAULT_PORT,
            )
        )


def _make_ros_client(distro_label, port_env, default_port, legacy_port_env=None):
    _skip_if_not_opted_in()

    from compas_fab.backends import RosClient

    host = os.environ.get(HOST_ENV_VAR, "localhost")
    if legacy_port_env and legacy_port_env in os.environ:
        port = int(os.environ[legacy_port_env])
    else:
        port = int(os.environ.get(port_env, default_port))

    client = RosClient(host=host, port=port)
    try:
        client.run(timeout=5)
    except Exception as e:
        pytest.skip(
            "Could not connect to {0} rosbridge at {1}:{2}: {3}".format(distro_label, host, port, e)
        )

    return client


@pytest.fixture(scope="module")
def ros1_client():
    """Live RosClient connected to the ROS 1 / MoveIt 1 stack.

    Default host port is ``COMPAS_FAB_ROS1_PORT`` (9090). Skipped if
    ``COMPAS_FAB_RUN_ROS_INTEGRATION_TESTS != "1"`` or if rosbridge isn't
    reachable.
    """
    client = _make_ros_client(
        "ROS 1",
        ROS1_PORT_ENV_VAR,
        ROS1_DEFAULT_PORT,
        legacy_port_env=LEGACY_PORT_ENV_VAR,
    )
    yield client
    client.close()


@pytest.fixture(scope="module")
def ros2_client():
    """Live RosClient connected to the ROS 2 / MoveIt 2 stack.

    Default host port is ``COMPAS_FAB_ROS2_PORT`` (9091). Skipped if
    ``COMPAS_FAB_RUN_ROS_INTEGRATION_TESTS != "1"`` or if rosbridge isn't
    reachable.
    """
    client = _make_ros_client("ROS 2", ROS2_PORT_ENV_VAR, ROS2_DEFAULT_PORT)
    yield client
    client.close()
