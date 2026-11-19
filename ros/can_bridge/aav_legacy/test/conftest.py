"""
The chain tests' bridge (and rc_driver, and muxes) in either language.

Python (aav_legacy, aav_rc, aav_io_mux): the node in the test's own process, spun with it.
Rust (aav_legacy_rs, aav_rc_rs, aav_io_mux_rs, the defaults since 17 Nov): the built binary
in its own process group, on the test's ROS domain and this computer only, stopped by its
process group. The `impl` fixture runs a test once with each; Rust is skipped where its
packages aren't built.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys

import pytest

# The adaptive brake's tests (test_uart_relay.py) import aav_description from the source
# tree, so tools/test.sh runs them without a built workspace.
DESCRIPTION = Path(__file__).resolve().parents[3] / 'bringup' / 'aav_description'
if str(DESCRIPTION) not in sys.path:
    sys.path.append(str(DESCRIPTION))

try:
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError
except ImportError:
    get_package_prefix = None


def rust_binary(package, executable):
    """Return the installed Rust binary's path, or None if it isn't built here."""
    if get_package_prefix is None:
        return None
    try:
        path = os.path.join(get_package_prefix(package), 'lib', package, executable)
    except PackageNotFoundError:
        return None
    return path if os.path.exists(path) else None


class RustNode:
    """A Rust node in its own process group; destroy_node() stops it as rclpy's would."""

    def __init__(self, package, executable, args, domain):
        exe = rust_binary(package, executable)
        if exe is None:
            pytest.skip(f'{package} is not built here')
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain),
                   ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        self.process = subprocess.Popen([exe, *args], env=env, start_new_session=True,
                                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def destroy_node(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()


def ros_nodes(nodes):
    """Return the nodes that run in this process (to add to the executor)."""
    return [n for n in nodes if not isinstance(n, RustNode)]


@pytest.fixture(params=['python', 'rust'])
def impl(request):
    """Each chain test once with the Python nodes, once with the Rust ones."""
    if request.param == 'rust' and rust_binary('aav_legacy_rs', 'legacy_uart_bridge') is None:
        pytest.skip('aav_legacy_rs is not built here')
    return request.param
