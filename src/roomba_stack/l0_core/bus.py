from __future__ import annotations

from roomba_stack.l2_oi.protocol_queue import BoundedQueue

from typing import Any, Callable, Dict, Type, TypeVar, DefaultDict, List
from collections import defaultdict
import threading
import queue

T = TypeVar("T")


class CommandBus:
    """
    Minimal synchronous command router.

    Each command class (StopCmd, DriveCmd, etc.) can have exactly one handler. `call`
    executes the bound handler on the caller's thread so upstream layers stay in the
    Half-Sync / Half-Async model—command producers do not touch `OIService` directly.
    """

    def __init__(self) -> None:
        self._handlers: Dict[Type[Any], Callable[[Any], Any]] = {}

    def register(self, command_type: Type[T], handler: Callable[[T], Any]) -> None:
        """
        Bind ``handler`` to ``command_type``.

        Parameters
        ----------
        command_type : Type[T]
            The dataclass/type representing the command.
        handler : Callable[[T], Any]
            Callable invoked whenever ``command_type`` is dispatched. Runs on
            the caller's thread; must be thread-safe or delegate as needed.

        Raises
        ------
        ValueError
            If ``command_type`` is already registered.
        """
        if command_type in self._handlers:
            raise ValueError(f"Handler already registered for {command_type.__name__}")
        self._handlers[command_type] = handler

    def call(self, command: T) -> Any:
        """
        Execute the handler for ``command`` synchronously.

        Parameters
        ----------
        command : T
            Concrete command instance to route. Its type determines which
            handler is selected.

        Raises
        ------
        LookupError
            If no handler has been registered for ``type(command)``.
        """
        command_type = type(command)
        handler = self._handlers.get(command_type)
        if handler is None:
            raise LookupError(f"No handler registered for {command_type.__name__}")
        return handler(command)

class EventBus:
    """
    Lightweight in-process pub/sub.

    * Bounded queue prevents unbounded growth (drop-newest policy).
    * Single dispatcher thread delivers events serially to all subscribers.
    * Subscribers run on the dispatcher thread, so keep handlers non-blocking.
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
        """
        Register ``callback`` to receive events for ``topic``.

        Parameters
        ----------
        topic : str
            Topic name (e.g., "sensors", "voice.transcript").
        callback : Callable[[object], None]
            Function invoked with the event payload. Runs on the dispatcher
            thread; keep it fast or offload work internally.
        """
        self._subscribers[topic].append(callback)

    def unsubscribe(self, topic: str, callback: Callable[[object], None]) -> None:
        """
        Remove ``callback`` from ``topic`` if previously subscribed.

        Parameters
        ----------
        topic : str
            Topic name.
        callback : Callable[[object], None]
            Callback to remove.
        """
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

                    
                    
