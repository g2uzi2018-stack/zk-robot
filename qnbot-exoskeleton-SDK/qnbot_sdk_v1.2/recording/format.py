from __future__ import annotations

from dataclasses import dataclass


@dataclass
class SerialReplayRecord:
    timestamp_ns: int
    chunk_bytes: bytes
