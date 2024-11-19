import fnmatch
import io
import os
import re

from sybil import Region
from sybil import Sybil
from sybil.parsers.codeblock import PythonCodeBlockParser
from sybil.parsers.doctest import DocTestParser

LITERALPYTHON_START = re.compile(r"\.\.\s*literalinclude\s*::(.*\.py)")
LITERALPYTHON_END = re.compile(r"(\n\Z|\n(?=\S))")
IGNORES = [
    "files/01_ros_*.py",
    "files/03_robot_rhino*.py",
    "files/04_cartesian_path_analytic_pybullet.py",
]


def evaluate_literalinclude_python_blocks(block_match):
    source_path = os.path.dirname(block_match.path)
    literalinclude_python = os.path.join(source_path, block_match.parsed)

    with io.open(literalinclude_python, "r", encoding="utf-8") as f:
        filecontent = f.read()
        exec(filecontent)


def parse_literalinclude_python_blocks(document):
    for start_match, end_match, source in document.find_region_sources(LITERALPYTHON_START, LITERALPYTHON_END):
        literalinclude_python = start_match.groups()[0].strip()

        if any([fnmatch.fnmatch(literalinclude_python, ignore_pattern) for ignore_pattern in IGNORES]):
            continue

        print(literalinclude_python)
        yield Region(start_match.start(), end_match.end(), literalinclude_python, evaluate_literalinclude_python_blocks)


pytest_collect_file = Sybil(
    parsers=[DocTestParser(), PythonCodeBlockParser(), parse_literalinclude_python_blocks],
    pattern="*.rst",
    fixtures=[],
    excludes=[],
).pytest()
