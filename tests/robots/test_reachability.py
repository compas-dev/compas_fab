import math
import os

from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Vector
from compas.geometry import allclose

from compas_fab.backends import AnalyticalInverseKinematics
from compas_fab.robots import ReachabilityMap
from compas_fab.robots.reachability_map import DeviationVectorsGenerator
from compas_fab.robots.reachability_map import OrthonormalVectorsFromAxisGenerator

filename = os.path.join(os.path.dirname(__file__), "fixtures", "map.json")


def test_vector_generators():
    generator = OrthonormalVectorsFromAxisGenerator((0, 0, 1), math.radians(120))
    result = [xaxis for xaxis in generator]
    assert allclose(result[0], Vector(0.000, -1.000, 0.000), tol=1e-3)
    assert allclose(result[1], Vector(0.866, 0.500, 0.000), tol=1e-3)
    assert allclose(result[2], Vector(-0.866, 0.500, 0.000), tol=1e-3)

    generator = DeviationVectorsGenerator((0, 0, -1), math.radians(120), 1)
    result = [zaxis for zaxis in generator]
    assert allclose(result[0], Vector(0.000, 0.000, -1.000), tol=1e-3)
    assert allclose(result[1], Vector(-0.866, 0.000, 0.500), tol=1e-3)
    assert allclose(result[2], Vector(0.433, 0.750, 0.500), tol=1e-3)
    assert allclose(result[3], Vector(0.433, -0.750, 0.500), tol=1e-3)


def test_reachability_scores():
    map = ReachabilityMap.from_json(filename)

    assert allclose(map.score, [0, 38, 102, 145, 132, 137])
    assert allclose(map.best_score, (145, 3))


if __name__ == "__main__":
    from compas_fab.backends import PyBulletClient

    def frame_generator(pt):
        zaxis = Vector(0, 0, 1)
        for axis in DeviationVectorsGenerator(zaxis, math.radians(40), 1):
            for xaxis in OrthonormalVectorsFromAxisGenerator(axis, math.radians(60)):
                yaxis = axis.cross(xaxis)
                yield Frame(pt, xaxis, yaxis)

    def generator():
        for i in range(6):
            pt = Point(0, 0, 0) + Vector(0, i * 0.1, 0)
            yield frame_generator(pt)

    with PyBulletClient(connection_type="direct") as client:
        robot = client.load_ur5(load_geometry=True)
        ik = AnalyticalInverseKinematics(client)
        client.inverse_kinematics = ik.inverse_kinematics
        options = {"solver": "ur5", "check_collision": True, "keep_order": True}

        map = ReachabilityMap()
        map.calculate(generator(), robot, options)
        map.to_json(filename)
