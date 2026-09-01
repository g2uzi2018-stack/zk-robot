from __future__ import annotations

import time
from pathlib import Path
from typing import Iterator, Optional

from qnbot_sdk import QnbotStreamParser, TelemetrySnapshot
from recording.reader import read_records

from ._client_surface import TelemetryClientSurface, install_qnbot_client_surface


@install_qnbot_client_surface
class ReplayClient(TelemetryClientSurface):
    """Replay timestamped raw serial chunks through the normal parser."""

    def __init__(self, path: str | Path, speed: float = 1.0):
        self.path = Path(path)
        self.speed = max(float(speed), 1e-6)
        self._iterator: Optional[Iterator[TelemetrySnapshot]] = None
        self._running = False
        self._eof = False
        self._latest_telemetry: Optional[TelemetrySnapshot] = None

    def start(self) -> None:
        self._iterator = self._iter_snapshots()
        self._running = True
        self._eof = False

    def stop(self) -> None:
        self._running = False
        self._iterator = None

    def get_latest_telemetry(self) -> Optional[TelemetrySnapshot]:
        if self._eof:
            return None
        if self._iterator is None:
            self.start()
        assert self._iterator is not None
        try:
            self._latest_telemetry = next(self._iterator)
            return self._latest_telemetry
        except StopIteration:
            self._eof = True
            self.stop()
            return None

    def _iter_snapshots(self) -> Iterator[TelemetrySnapshot]:
        parser = QnbotStreamParser()
        previous_ns: Optional[int] = None
        for record in read_records(self.path):
            if not self._running:
                return
            if previous_ns is not None:
                delay = (record.timestamp_ns - previous_ns) / 1_000_000_000.0 / self.speed
                if delay > 0:
                    time.sleep(delay)
            previous_ns = record.timestamp_ns
            for item in parser.feed(record.chunk_bytes):
                if isinstance(item, TelemetrySnapshot):
                    yield item
