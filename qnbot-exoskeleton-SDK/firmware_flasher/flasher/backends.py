from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path


class FlashBackend(ABC):
    @abstractmethod
    def list_devices(self) -> str:
        pass

    @abstractmethod
    def flash(self, firmware: Path, **kwargs: object) -> str:
        pass
