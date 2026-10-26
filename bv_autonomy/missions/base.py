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
        os.replace(tmp, self.path)

    @classmethod
    def load(cls) -> Self:
        return cls.from_dict(json.loads(cls.path.read_text()))

    @classmethod
    @abstractmethod
    def from_dict(cls, data: dict) -> Self:
        """Rebuilds a state from its saved form."""


class Mission(ABC):
    """One competition. Subclasses build their components in __init__ and write run()."""

    state: MissionState

    async def start(self):
        """Runs the mission, and gets the aircraft somewhere safe if it fails or is stopped."""
        try:
            await self.run()
        except BaseException:
            await self.on_abort()
            raise

    @abstractmethod
    async def run(self):
        """Flies the competition from start to finish."""

    async def on_abort(self):
        """Gets the aircraft somewhere safe after the mission fails or is stopped."""
