"""In-memory ring buffer feeding the log-only UI. No graphics, just logs."""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass


@dataclass
class LogEntry:
    ts: float
    robot_id: str
    message: str


class LogBuffer:
    def __init__(self, maxlen: int = 500) -> None:
        self._entries: deque[LogEntry] = deque(maxlen=maxlen)

    def add(self, robot_id: str, message: str) -> None:
        self._entries.append(LogEntry(ts=time.time(), robot_id=robot_id, message=message))

    def tail(self, n: int = 200) -> list[dict]:
        return [{"ts": e.ts, "robot_id": e.robot_id, "message": e.message} for e in list(self._entries)[-n:]]
