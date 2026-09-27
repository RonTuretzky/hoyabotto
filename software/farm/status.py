"""One status vocabulary for every device and every derived value.

Nothing above the adapter layer ever sees a bare number: it sees a Reading,
and it must check `status` before using `value`.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Generic, TypeVar

T = TypeVar("T")


class Status(str, Enum):
    OK = "OK"                          # fresh, valid, from an enabled device
    STALE = "STALE"                    # a value exists but is too old to act on
    INVALID = "INVALID"                # device present but the reading cannot be trusted
    NOT_APPLICABLE = "NOT_APPLICABLE"  # device disabled in the profile; never a reading
    UNKNOWN = "UNKNOWN"                # a derived judgement that could not be made


@dataclass
class Reading(Generic[T]):
    value: T | None
    status: Status
    t: float = field(default_factory=time.time)  # unix seconds when the value was captured
    seq: int | None = None                        # monotonically increasing per source, if the source has one
    source: str = ""                              # adapter / extractor name
    note: str = ""                                # why the status is not OK
    meta: dict[str, Any] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.status is Status.OK

    def age(self, now: float | None = None) -> float:
        return (now if now is not None else time.time()) - self.t

    def aged(self, max_age_s: float, now: float | None = None) -> "Reading[T]":
        """Return a copy downgraded to STALE if older than `max_age_s`."""
        if self.status is Status.OK and self.age(now) > max_age_s:
            return Reading(self.value, Status.STALE, self.t, self.seq, self.source, f"age {self.age(now):.2f}s > {max_age_s}s", dict(self.meta))
        return self

    def to_record(self) -> dict[str, Any]:
        v = self.value
        if hasattr(v, "shape"):  # numpy images are stored as files elsewhere, never inline
            v = {"shape": list(v.shape), "dtype": str(v.dtype)}
        return {"value": v, "status": self.status.value, "t": self.t, "seq": self.seq, "source": self.source, "note": self.note, "meta": self.meta}


def not_applicable(source: str, why: str = "disabled in profile") -> Reading[Any]:
    return Reading(None, Status.NOT_APPLICABLE, source=source, note=why)


def invalid(source: str, why: str) -> Reading[Any]:
    return Reading(None, Status.INVALID, source=source, note=why)


def unknown(source: str, why: str) -> Reading[Any]:
    return Reading(None, Status.UNKNOWN, source=source, note=why)
