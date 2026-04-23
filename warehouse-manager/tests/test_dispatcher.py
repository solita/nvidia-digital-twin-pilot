"""Basic tests for dispatcher logic."""

from dispatcher import nearest_assign, batch_assign


def test_nearest_assign_single():
    orders = [("order_1", (10.0, 5.0))]
    forklifts = [("fl_0", (0.0, 0.0)), ("fl_1", (9.0, 4.0))]
    result = nearest_assign(orders, forklifts)
    assert result == {"fl_1": "order_1"}


def test_nearest_assign_multiple():
    orders = [("o1", (0.0, 0.0)), ("o2", (10.0, 10.0))]
    forklifts = [("f0", (1.0, 1.0)), ("f1", (9.0, 9.0))]
    result = nearest_assign(orders, forklifts)
    assert result == {"f0": "o1", "f1": "o2"}


def test_nearest_assign_no_forklifts():
    result = nearest_assign([("o1", (0.0, 0.0))], [])
    assert result == {}


def test_batch_assign_single():
    orders = [("order_1", (10.0, 5.0))]
    forklifts = [("fl_0", (0.0, 0.0)), ("fl_1", (9.0, 4.0))]
    result = batch_assign(orders, forklifts)
    assert result == {"fl_1": "order_1"}


def test_batch_assign_optimal():
    # Two forklifts, two orders — Hungarian should find optimal pairing
    orders = [("o1", (0.0, 0.0)), ("o2", (10.0, 10.0))]
    forklifts = [("f0", (1.0, 1.0)), ("f1", (9.0, 9.0))]
    result = batch_assign(orders, forklifts)
    assert result == {"f0": "o1", "f1": "o2"}


def test_batch_assign_empty():
    assert batch_assign([], [("f0", (0.0, 0.0))]) == {}
    assert batch_assign([("o1", (0.0, 0.0))], []) == {}
