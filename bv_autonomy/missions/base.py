import json
import os
from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import ClassVar, Self


@dataclass
class MissionState(ABC):
    """What's left of a mission, saved as it changes so a stopped mission can resume.

    Subclasses add the fields, set path, and write from_dict.
    """

    path: ClassVar[Path]

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(asdict(self)))
        os.replace(tmp, self.path)       # a crash mid-write leaves the previous file whole

    @classmethod
    def load(cls) -> Self:
        return cls.from_dict(json.loads(cls.path.read_text()))

    @classmethod
    @abstractmethod
    def from_dict(cls, data: dict) -> Self:
        """Rebuild the fields from saved JSON, which keeps only plain lists and dicts."""


class Mission(ABC):
    """One competition. Subclasses build their components in __init__ and write run()."""

    state: MissionState

    async def start(self):
        """Every mission inherits this. Don't override it."""
        try:
            await self.run()
        except BaseException:               # also Ctrl+C, which arrives as a cancellation
            await self.on_abort()
            raise

    @abstractmethod
    async def run(self):
        """The competition, step by step."""

    async def on_abort(self):
        """Called when run() fails or is stopped. Override it to get the aircraft somewhere safe."""
