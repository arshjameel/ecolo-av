"""
A launch value must reach a Rust node (docs/languages.md, "Launch overrides").

Every Rust Node() with more than one parameter source has a namespace.

rclrs 0.8 reads a parameter file's entry under the node's own name AFTER every `/**`
entry, whatever the order of the files. launch_ros writes a launch dict under `/**` when
the Node has no namespace, so a value from launch (`max_speed:=0.3`, vehicle.yaml's
steering limit) loses to the package's YAML in Rust, and wins in Python. With a namespace
(`namespace='/'`: the same names) launch writes the dict under the node's full name too,
and the later file wins in both languages. Found 18 Nov (the lane finder's debug_image).

rclrs also matches only `/**` and the exact full name, never a wildcard such as
`/**/joy_alive`: a YAML for a Rust node names it in full.

No ROS: reads the launch files' source.
"""

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def rust_packages():
    """Return the ROS packages that build an rclrs node."""
    return {p.parent.name for p in ROOT.glob('ros/*/*/Cargo.toml')
            if 'rclrs' in p.read_text() or any(
                'rclrs' in c.read_text() for c in p.parent.glob('crates/*/Cargo.toml'))}


RUST = rust_packages()


def may_be_rust(source):
    """Return True if package= may name a Rust package (literal, or a *_IMPLS switch)."""
    text = source.strip('\'"')
    return text in RUST or 'IMPLS' in source or 'bridges.package' in source


def rust_nodes():
    """Yield (file:line, package source, keywords) for each Node() that may be Rust."""
    for path in sorted(ROOT.glob('ros/*/*/launch/*.py')):
        src = path.read_text()
        for call in ast.walk(ast.parse(src)):
            if not (isinstance(call, ast.Call)
                    and getattr(call.func, 'id', getattr(call.func, 'attr', None)) == 'Node'):
                continue
            kw = {k.arg: k.value for k in call.keywords if k.arg}
            if 'package' in kw and may_be_rust(ast.get_source_segment(src, kw['package'])):
                yield (f'{path.relative_to(ROOT)}:{call.lineno}',
                       ast.get_source_segment(src, kw['package']), kw)


def sources(params):
    """How many parameter sources: a list's entries, or None when it isn't a list."""
    if isinstance(params, (ast.List, ast.Tuple)):
        return sum(2 if isinstance(e, ast.Starred) else 1 for e in params.elts)
    if isinstance(params, ast.BinOp):   # files + [dicts]
        return 2
    return None


def test_the_rust_packages_are_found():
    assert {'aav_ego_rs', 'aav_legacy_rs', 'aav_io_mux_rs', 'aav_hud'} <= RUST


def test_every_rust_node_with_several_sources_has_a_namespace():
    bad = []
    for where, package, kw in rust_nodes():
        n = sources(kw.get('parameters'))
        if 'parameters' in kw and (n is None or n > 1) and 'namespace' not in kw:
            bad.append(f'{where} ({package})')
    assert not bad, ('Rust nodes whose launch values may lose to their YAML (add '
                     f"namespace='/': docs/languages.md, Launch overrides): {bad}")


def test_the_safety_nodes_are_seen():
    # The scan itself: these must be among the nodes it checks.
    seen = ' '.join(f'{w} {p}' for w, p, _ in rust_nodes())
    for part in ('legacy.launch.py', 'spine.launch.py', 'navigation.launch.py',
                 'watchdog.launch.py', 'hud.launch.py'):
        assert part in seen, part
