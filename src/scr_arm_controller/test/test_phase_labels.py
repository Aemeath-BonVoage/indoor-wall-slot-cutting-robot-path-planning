"""按时间标注阶段（slot_execution.label_by_time）的单元测试."""
from scr_arm_controller.slot_execution import label_by_time


def test_samples_take_last_event_not_later_than_them():
    events = [(1.0, 3, 'arrived'), (2.0, 3, 'move'), (3.5, 3, 'cut'), (5.0, 3, 'move'),
              (5.3, -1, 'idle')]
    times = [0.5, 1.0, 1.99, 2.0, 3.49, 3.5, 4.9, 5.0, 5.31, 9.0]
    assert label_by_time(times, events) == [
        (-1, 'idle'), (3, 'arrived'), (3, 'arrived'), (3, 'move'), (3, 'move'), (3, 'cut'),
        (3, 'cut'), (3, 'move'), (-1, 'idle'), (-1, 'idle')]


def test_late_delivery_does_not_change_labels():
    """按时间表发布的阶段即使晚于生效时刻才送达（列表里乱序），标注也不变."""
    ordered = [(2.0, 0, 'move'), (3.5, 0, 'cut'), (5.0, 0, 'move'), (5.3, -1, 'idle')]
    late = [ordered[0], ordered[3], ordered[1], ordered[2]]
    times = [k * 0.05 for k in range(140)]
    assert label_by_time(times, late) == label_by_time(times, ordered)
    assert label_by_time([4.0], late) == [(0, 'cut')]


def test_simultaneous_events_keep_their_order():
    events = [(1.0, 2, 'idle'), (1.0, -1, 'idle'), (1.0, -1, 'move')]
    assert label_by_time([1.0, 1.5], events) == [(-1, 'move'), (-1, 'move')]
    assert label_by_time([0.0], []) == [(-1, 'idle')]
