"""Tests for the clouds waiting for their transform (no ROS)."""

from aav_things.tf_wait import decide, Step, WaitShape, Waiting

SHAPE = WaitShape(tolerance=0.1, max_wait=1.5, size=3)


def test_a_cloud_with_its_transform_is_used_at_its_stamp():
    assert decide(10.0, None, True, 10.0, SHAPE) is Step.AT_STAMP


def test_a_hair_newer_than_tf_uses_the_newest_transform():
    assert decide(10.05, 10.0, False, 10.05, SHAPE) is Step.NEWEST


def test_the_seen_warning_waits_then_drops():
    # 14 Nov, our simulator: requested 91.5, the newest transform at 90.32.
    assert decide(91.5, 90.32, False, 91.5, SHAPE) is Step.WAIT
    assert decide(91.5, 90.32, False, 92.9, SHAPE) is Step.WAIT
    assert decide(91.5, 90.32, False, 93.1, SHAPE) is Step.DROP


def test_no_transform_yet_waits_then_drops():
    assert decide(5.0, None, False, 5.5, SHAPE) is Step.WAIT
    assert decide(5.0, None, False, 7.0, SHAPE) is Step.DROP


def test_older_than_the_buffer_is_dropped_at_once():
    assert decide(1.0, 20.0, False, 20.0, SHAPE) is Step.DROP


def test_clouds_come_out_oldest_first_once_tf_catches_up():
    waiting = Waiting(SHAPE)
    for stamp in (1.0, 1.1, 1.2):
        waiting.add(stamp, f'cloud {stamp}')
    tf_until = [0.5]

    def look(stamp, _item):
        return stamp <= tf_until[0], tf_until[0]

    assert waiting.take(look, now=1.2) == []          # all too new: none taken, none lost
    assert len(waiting.queue) == 3
    tf_until[0] = 1.1
    assert waiting.take(look, now=1.3) == [('cloud 1.0', Step.AT_STAMP),
                                           ('cloud 1.1', Step.AT_STAMP),
                                           ('cloud 1.2', Step.NEWEST)]
    assert not waiting.queue and waiting.dropped == 0


def test_a_full_queue_drops_its_oldest_and_counts():
    waiting = Waiting(SHAPE)
    for stamp in (1.0, 1.1, 1.2, 1.3):
        waiting.add(stamp, stamp)
    assert [s for s, _ in waiting.queue] == [1.1, 1.2, 1.3]
    assert waiting.dropped == 1


def test_a_stale_cloud_is_dropped_and_the_next_still_waits():
    waiting = Waiting(SHAPE)
    waiting.add(1.0, 'old')
    waiting.add(2.5, 'new')
    ready = waiting.take(lambda stamp, item: (False, 0.0), now=2.6)
    assert ready == [] and waiting.dropped == 1
    assert [item for _, item in waiting.queue] == ['new']
