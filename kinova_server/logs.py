"""Logging setup + the WebSocket log broadcaster.

A :class:`logging.Handler` pushes formatted records onto a thread-safe
``queue.Queue``. A single asyncio pump (started in the app lifespan) drains it
by *blocking* in an executor thread — no 100 ms busy poll (PLAN 2.6) — and
fans each line out to every connected ``/ws/logs`` subscriber.
"""

from __future__ import annotations

import asyncio
import logging
import queue as stdlib_queue
from typing import Set

# Thread-safe sink the logging handler writes to.
_log_queue: "stdlib_queue.Queue[str]" = stdlib_queue.Queue(maxsize=1000)


class _WsBroadcastHandler(logging.Handler):
    def emit(self, record: logging.LogRecord) -> None:
        msg = f"[{record.levelname}] {self.format(record)}"
        try:
            _log_queue.put_nowait(msg)
        except stdlib_queue.Full:
            pass


def setup_logging() -> logging.Logger:
    handler = _WsBroadcastHandler()
    handler.setFormatter(logging.Formatter("%(message)s"))
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s — %(message)s",
        handlers=[logging.StreamHandler(), handler],
    )
    return logging.getLogger("kinova-server")


# ── Broadcaster ────────────────────────────────────────────────────────────
_subscribers: Set["asyncio.Queue[str]"] = set()


def subscribe() -> "asyncio.Queue[str]":
    q: "asyncio.Queue[str]" = asyncio.Queue(maxsize=300)
    _subscribers.add(q)
    return q


def unsubscribe(q: "asyncio.Queue[str]") -> None:
    _subscribers.discard(q)


async def log_pump() -> None:
    """Block on the stdlib queue in a worker thread; fan out each line.

    Blocking ``Queue.get`` in an executor means we never spin when idle and
    deliver immediately when a record arrives.
    """
    loop = asyncio.get_running_loop()
    while True:
        msg = await loop.run_in_executor(None, _log_queue.get)
        for q in list(_subscribers):
            try:
                q.put_nowait(msg)
            except asyncio.QueueFull:
                pass
