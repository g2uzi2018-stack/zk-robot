from __future__ import annotations

import inspect

from qnbot_sdk import QnbotClient


class TelemetryClientSurface:
    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    def get_latest_data(self):
        return self.get_latest_telemetry()


def _make_noop_method(name: str):
    def method(self, *args, **kwargs):
        del args, kwargs
        return None

    method.__name__ = name
    return method


def install_qnbot_client_surface(cls):
    for name, value in inspect.getmembers(QnbotClient, inspect.isfunction):
        if name.startswith("_") or hasattr(cls, name):
            continue
        setattr(cls, name, _make_noop_method(name))
    return cls
