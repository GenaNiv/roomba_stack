from typing import Any, Tuple, Literal
from queue import Queue, Full, Empty


class BoundedQueue:
    """
    Bounded, thread-safe queue wrapper with:
      - fixed capacity (maxsize)
      - explicit overflow policy (default: 'drop_newest')
      - timed put/get (timeouts passed per call)
    """

    def __init__(self, maxsize: int, name: str,
                 on_overflow: Literal["drop_newest"] = "drop_newest") -> None:
        """TBD"""
        if not isinstance(maxsize, int) or maxsize <= 0:
            raise ValueError("maxsize must be a positive integer")
        if not isinstance(name, str) or not name.strip():
            raise ValueError("name must be a non-empty string")
        if not isinstance(on_overflow, str) or on_overflow != "drop_newest":
            raise ValueError("on_overflow must be 'drop_newest'")
        self._name = name
        self._maxsize = maxsize
        self._on_overflow = on_overflow
        self._q = Queue(maxsize=self._maxsize)

        
        
    def put(self, item: Any, timeout: float) -> bool:
        """Return True on enqueue; False if overflow policy drops the item."""
        try:
            self._q.put(item, timeout=timeout)
            return True
        except Full:
            return False

    def get(self, timeout: float) -> Tuple[bool, Any]:
        """Return (True, item) on success; (False, None) on timeout."""
        try:
            item = self._q.get(timeout=timeout)
            return True, item
        except Empty:
            return False, None

    def qsize(self) -> int:
        """Current number of items in the queue (approximate)."""
        return self._q.qsize()

    def maxsize(self) -> int:
        """Configured capacity of the queue."""
        return self._maxsize

    def name(self) -> str:
        """Returns the name of the queue."""
        return self._name
