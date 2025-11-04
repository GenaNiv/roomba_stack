from __future__ import annotations

from roomba_stack.l2_oi.protocol_queue import BoundedQueue

from typing import Any, Callable, Dict, Type, TypeVar, DefaultDict, List
from collections import defaultdict
import threading
import queue

T = TypeVar("T")


class CommandBus:
    """
    Simple, in-process command router.

    Usage:
      bus = CommandBus()
      bus.register(StopCmd, handler)
      result = bus.call(StopCmd(reason="user"))
    """

    def __init__(self) -> None:
        # One handler per command type
        self._handlers: Dict[Type[Any], Callable[[Any], Any]] = {}

    def register(self, command_type: Type[T], handler: Callable[[T], Any]) -> None:
        """Associate a command class with its handler callable."""
        if command_type in self._handlers:
            raise ValueError(f"Handler already registered for {command_type.__name__}")
        self._handlers[command_type] = handler

    def call(self, command: T) -> Any:
        """Synchronously execute the handler for the given command instance."""
        command_type = type(command)
        handler = self._handlers.get(command_type)
        if handler is None:
            raise LookupError(f"No handler registered for {command_type.__name__}")
        # Synchronous call: caller waits for the handler to finish and return a result.
        return handler(command)

class EventBus:
    """
    In-process publish/subscribe.

    - A bounded queue prevents unbounded memory growth during bursts.
    - One dispatcher thread delivers events in order to all subscribers.
    - All subscriber callbacks run on the dispatcher thread (no data races among listeners).
    """

    def __init__(self, capacity: int = 1024, publish_timeout_ms: int = 10) -> None:
        self._subscribers: DefaultDict[str, List[Callable[[object], None]]] = defaultdict(list)
        self._queue: BoundedQueue = BoundedQueue(maxsize=capacity, name="EventBus")
        self._publish_timeout = publish_timeout_ms / 1000.0
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="EventBus-Dispatcher", daemon=True
        )
        self._thread.start()

    # ---- subscription API ----
    def subscribe(self, topic: str, callback: Callable[[object], None]) -> None:
        self._subscribers[topic].append(callback)

    def unsubscribe(self, topic: str, callback: Callable[[object], None]) -> None:
        if callback in self._subscribers[topic]:
            self._subscribers[topic].remove(callback)

    # ---- publishing API ----
    def publish(self, topic: str, event: object) -> bool:
        """
        Short-wait publish. Returns True if enqueued; False if the queue was full.

        Policy for now: drop newest when full (protects the producer thread).
        We can add a coalescing or drop-oldest policy later as a constructor option.
        """
        return self._queue.put((topic, event), timeout=self._publish_timeout)

    # ---- lifecycle ----
    def close(self) -> None:
        # best-effort sentinel; if full, we’ll exit via timeout below
        self._queue.put(("__stop__", None), timeout=0.0)
        self._thread.join(timeout=1.0)

    # ---- dispatcher loop ----
    def _run(self) -> None:
        while not self._stop.is_set():
            ok, item = self._queue.get(timeout=0.1)
            if not ok:
                continue
            topic, event = item
            if topic == "__stop__":
                break
            for callback in list(self._subscribers.get(topic, [])):
                try:
                    callback(event)  # all callbacks run on this single dispatcher thread
                except Exception as ex:
                    # Placeholder: in a later step we will publish a Fault event instead of print
                    print(f"[EventBus] subscriber error on topic '{topic}': {ex}")

                    
                    