"""
A ROS domain of its own for each pytest run inside `pixi run test` (tools/colcon_test.sh).

colcon test runs several packages at once. Two packages' tests on one ROS domain hear each
other's nodes (24 Oct: the HUD's e-stop found a listener, the guard test saw street_router,
the spine bridge test another package's /vehicle/odom). At the start of each pytest run
this takes a domain no other process on this computer holds (domain_coordinator, ROS's
own: a port per domain, 1 to 100, held until pytest ends) and sets ROS_DOMAIN_ID, so
every node the tests start, in this process or as a child, is on it. AAV_TEST_DOMAIN says
the same to tests that otherwise choose their own domain: they take it instead.

Loaded by `-p aav_ros_domain` with this folder on PYTHONPATH; nothing else loads it, so
plain pytest and tools/test.sh are unchanged.
"""

import contextlib
import os

import domain_coordinator

_held = contextlib.ExitStack()


def pytest_configure(config):
    """Take a free domain for this run and put the run on it."""
    domain = _held.enter_context(domain_coordinator.domain_id())
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['AAV_TEST_DOMAIN'] = str(domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'


def pytest_report_header(config):
    """Say which domain the run is on."""
    return f"ROS_DOMAIN_ID {os.environ.get('AAV_TEST_DOMAIN')} (aav_ros_domain)"


def pytest_unconfigure(config):
    """Give the domain back."""
    _held.close()
