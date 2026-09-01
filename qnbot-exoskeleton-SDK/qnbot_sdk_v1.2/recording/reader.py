from __future__ import annotations

import base64
import json
from pathlib import Path
from typing import Iterator

from .format import SerialReplayRecord


def read_records(path: str | Path) -> Iterator[SerialReplayRecord]:
    with Path(path).open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            item = json.loads(line)
            yield SerialReplayRecord(
                timestamp_ns=int(item["timestamp_ns"]),
                chunk_bytes=base64.b64decode(item["chunk_b64"]),
            )
