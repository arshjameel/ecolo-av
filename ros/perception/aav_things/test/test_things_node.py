"""The node's message parts (needs the built workspace: colcon test)."""

import pytest

try:
    from aav_interfaces.msg import Thing, Things
    from aav_things import things_node
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='needs the built workspace')


def test_the_module_sends_the_message_type():
    # The node's class once shadowed the message's name: publish then built a node.
    assert things_node.Things is Things


def test_markers_a_box_an_arrow_and_a_label_per_thing():
    msg = Things()
    msg.header.frame_id = 'odom'
    msg.things = [Thing(id=4, x=10.0, y=1.0, vx=1.0, length=1.0, width=1.0, height=1.0,
                        radar=True)]
    markers = things_node.markers(msg).markers
    assert len(markers) == 4                     # DELETEALL, then three
    assert markers[0].action == markers[0].DELETEALL
    assert markers[3].text == '4 1.0 m/s R'
    assert markers[2].points[1].x == pytest.approx(11.0)
