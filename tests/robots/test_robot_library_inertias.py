from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

import compas_fab


ROBOT_LIBRARY = Path(compas_fab.get("robot_library"))
URDF_PATHS = sorted(ROBOT_LIBRARY.rglob("*.urdf"))


@pytest.mark.parametrize("urdf_path", URDF_PATHS, ids=lambda path: path.relative_to(ROBOT_LIBRARY).as_posix())
def test_bundled_inertias_are_physically_realisable(urdf_path):
    """Principal moments must be positive and satisfy the triangle inequality."""
    for link in ET.parse(urdf_path).getroot().findall("link"):
        inertia = link.find("inertial/inertia")
        if inertia is None:
            continue

        values = {key: float(inertia.attrib[key]) for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
        tensor = np.array(
            [
                [values["ixx"], values["ixy"], values["ixz"]],
                [values["ixy"], values["iyy"], values["iyz"]],
                [values["ixz"], values["iyz"], values["izz"]],
            ]
        )
        assert np.isfinite(tensor).all(), link.attrib["name"]
        moments = np.linalg.eigvalsh(tensor)
        context = "{}: principal moments {}".format(link.attrib["name"], moments)
        assert moments[0] > 0, context
        # eigvalsh sorts the moments; checking the largest covers all three inequalities.
        assert moments[0] + moments[1] >= moments[2] - 1e-9 * moments[2], context
