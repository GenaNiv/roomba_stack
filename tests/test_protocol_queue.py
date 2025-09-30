from roomba_stack.l2_oi.protocol_queue import BoundedQueue


def test_bounded_queue_timed_put_get():
    q = BoundedQueue(maxsize=2, name="tq")

    # Fill within capacity
    assert q.put(1, timeout=0.01) is True
    assert q.put(2, timeout=0.01) is True

    # Exceed capacity → dropped by policy (False)
    assert q.put(3, timeout=0.01) is False

    # Drain two items in FIFO order
    ok, val = q.get(timeout=0.01)
    assert ok is True and val == 1

    ok, val = q.get(timeout=0.01)
    assert ok is True and val == 2

    # Third get times out → (False, None)
    ok, val = q.get(timeout=0.01)
    assert ok is False and val is None
