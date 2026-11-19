"""
The nodes' attribute names don't hide their methods (no ROS: read from the source).

ActuatorTest.__init__ sets `self.estop` (the bool last heard on /safety/estop). Until
19 Nov PreflightNode's method that presses the e-stop was also called `estop`, so the
instance's bool hid it, and the preflight's last resort ("NOT STOPPED: E-STOP NOW")
raised TypeError instead of publishing /safety/estop true.
"""

import ast
from pathlib import Path

PKG = Path(__file__).resolve().parents[1] / 'aav_actuator_tests'


def attributes_set(cls):
    """Every `self.<name> = ...` in a class's methods."""
    out = set()
    for node in ast.walk(cls):
        if isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            for t in targets:
                for el in ast.walk(t):
                    if (isinstance(el, ast.Attribute) and isinstance(el.value, ast.Name)
                            and el.value.id == 'self'):
                        out.add(el.attr)
    return out


def classes(path):
    tree = ast.parse(path.read_text())
    return {n.name: n for n in tree.body if isinstance(n, ast.ClassDef)}


def methods(cls):
    return {n.name for n in cls.body if isinstance(n, ast.FunctionDef)}


def test_no_attribute_hides_a_method():
    found = {}
    for name in ('actuator_test.py', 'preflight_node.py', 'car_test_node.py'):
        if (PKG / name).exists():
            found.update(classes(PKG / name))
    chain = [found[c] for c in ('ActuatorTest', 'PreflightNode', 'CarTestNode') if c in found]
    attrs = set().union(*(attributes_set(c) for c in chain))
    meths = set().union(*(methods(c) for c in chain))
    assert not attrs & meths, f'an attribute hides a method: {sorted(attrs & meths)}'


def test_the_preflight_presses_with_press_estop():
    text = (PKG / 'preflight_node.py').read_text()
    assert 'node.press_estop()' in text and 'node.estop()' not in text
