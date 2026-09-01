from __future__ import annotations

import base64
import json
from pathlib import Path

from .format import SerialReplayRecord


def write_record(path: str | Path, record: SerialReplayRecord) -> None:
    item = {
        "timestamp_ns": int(record.timestamp_ns),
        "chunk_b64": base64.b64encode(record.chunk_bytes).decode("ascii"),
    }
    with Path(path).open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(item, separators=(",", ":")) + "\n")
