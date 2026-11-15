"""
A package the car install skips must have nothing in the workspace depending on it.

tools/car_install.sh skips some packages on the car (SKIP_PACKAGES, e.g. aav_hud) unless
--all is given: the screens are the dash's, not every computer's. colcon then fails any
workspace package that DEPENDS on a skipped one, because it can't source the skipped
package's environment (install/<pkg>/share/<pkg>/package.sh is missing). So a skipped
package must have no dependents inside the workspace.

car_install.sh's own comment states this ("Only a package nothing else depends on can be
skipped: colcon then fails its dependents"). CI builds the kit with --all, so it never
exercises the default skip path; this test does, against the package.xml graph, so a new
dependency on a skipped package is caught before it breaks the car build.
"""

from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
CAR_INSTALL = ROOT / 'tools' / 'car_install.sh'
# The dependency tags colcon reads when it sets up a package's build environment.
DEP_TAGS = ('depend', 'build_depend', 'buildtool_depend', 'build_export_depend',
            'exec_depend', 'test_depend')


def skip_packages():
    m = re.search(r'^SKIP_PACKAGES="([^"]*)"', CAR_INSTALL.read_text(), re.M)
    assert m, 'could not find SKIP_PACKAGES="..." in tools/car_install.sh'
    return m.group(1).split()


def workspace_packages():
    """Map each package name to its dependency names, for every package.xml under ros/."""
    pkgs = {}
    for px in sorted(ROOT.glob('ros/**/package.xml')):
        root = ET.parse(px).getroot()
        name = (root.findtext('name') or '').strip()
        assert name, f'{px} has no <name>'
        deps = {d.text.strip() for tag in DEP_TAGS for d in root.iter(tag) if d.text}
        pkgs[name] = deps
    return pkgs


def test_skip_packages_are_defined():
    assert skip_packages(), 'SKIP_PACKAGES is empty: nothing to check'


def test_skipped_packages_have_no_workspace_dependents():
    skip = skip_packages()
    pkgs = workspace_packages()
    assert pkgs, 'no package.xml files found under ros/'

    offenders = {}
    for s in skip:
        dependents = sorted(n for n, deps in pkgs.items() if n != s and s in deps)
        if dependents:
            offenders[s] = dependents

    assert not offenders, (
        'tools/car_install.sh skips these packages by default (no --all), but other\n'
        'workspace packages depend on them, so the default car build fails to build the\n'
        'dependents (colcon cannot source the skipped package):\n'
        + '\n'.join(f'  {s}  <- depended on by: {", ".join(deps)}'
                    for s, deps in offenders.items())
        + "\nFix: drop the dependency in the dependent's package.xml, or stop skipping the "
          'package in tools/car_install.sh (SKIP_PACKAGES).')
