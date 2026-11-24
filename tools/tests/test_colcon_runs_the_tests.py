"""
colcon test runs every test that tools/test.sh's pytest loop runs.

Inside `pixi run test` the loop leaves the packages' Python tests to colcon test, so a test
colcon never runs would run nowhere on Linux and ARM. An ament_python package's tests are
its whole test/ folder, as in the loop. An ament_cmake package runs only the files its
CMakeLists.txt names (ament_add_pytest_test): until 24 Nov, three of aav_description's
seven (the adaptive brake, the lane map, the stop law) ran only in the loop.

No ROS: reads tools/test.sh, package.xml and CMakeLists.txt.
"""

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]


def loop_packages():
    """Return the package folders tools/test.sh's pytest loop runs."""
    text = (ROOT / 'tools/test.sh').read_text()
    block = text[text.index('\nfor pkg in '):]
    return re.findall(r'ros/[a-z0-9_/]+', block[:block.index('; do')])


def build_type(package):
    """Return the package's build type: ament_python, ament_cmake..."""
    found = re.search(r'<build_type>\s*([a-z_]+)', (package / 'package.xml').read_text())
    return found.group(1) if found else None


def cmake_packages():
    """Return every ament_cmake package colcon builds."""
    return [p.parent for p in ROOT.glob('ros/**/package.xml')
            if build_type(p.parent) == 'ament_cmake'
            and not (p.parent / 'COLCON_IGNORE').exists()]


def test_the_loop_is_found():
    assert len(loop_packages()) > 20


def test_colcon_tests_every_package_of_the_loop():
    for rel in loop_packages():
        package = ROOT / rel
        assert (package / 'package.xml').is_file(), f'{rel}: not a ROS package'
        assert not (package / 'COLCON_IGNORE').exists(), f'{rel}: colcon skips it'
        assert build_type(package) in ('ament_python', 'ament_cmake'), rel


def test_every_test_file_of_a_cmake_package_is_registered():
    missing = []
    for package in cmake_packages():
        cmake = (package / 'CMakeLists.txt').read_text()
        for test in sorted(package.glob('test/test_*.py')):
            if f'test/{test.name}' not in cmake:
                missing.append(str(test.relative_to(ROOT)))
    assert not missing, (
        'colcon test never runs these: name each in its CMakeLists.txt '
        f'(ament_add_pytest_test): {missing}')
