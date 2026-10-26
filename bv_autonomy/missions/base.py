import json
import logging
import os
import sys
from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import ClassVar, Self


log = logging.getLogger(__name__)
_handlers: list[logging.Handler] = []


class _Console(logging.StreamHandler):
    """Writes to whatever the standard error stream is at the time of each message."""

    @property
    def stream(self):
        """Returns the current standard error stream."""
        return sys.stderr

    @stream.setter
    def stream(self, value):
        pass


class _ConsoleFormat(logging.Formatter):
    """Shows the time and message, naming the level only for warnings and errors and the source only outside our code."""

    def format(self, record: logging.LogRecord) -> str:
        """Returns the record as one short console line, followed by any traceback."""
        level = f"{record.levelname} " if record.levelno >= logging.WARNING else ""
        source = "" if record.name.startswith("bv_autonomy") else f"{record.name}: "
        line = f"{self.formatTime(record, '%H:%M:%S')}  {level}{source}{record.getMessage()}"
        if record.exc_info:
            line += "\n" + self.formatException(record.exc_info)
        return line


class _FileFormat(logging.Formatter):
    """Shows the full time, the level and the source, naming our modules by their last part."""

    def format(self, record: logging.LogRecord) -> str:
        """Returns the record as one log file line, followed by any traceback."""
        record.source = record.name.rsplit(".", 1)[-1] if record.name.startswith("bv_autonomy") else record.name
        return super().format(record)


def start_logging(directory: Path) -> Path:
    """Prints the mission's progress to the console and writes it with more detail to a log file in the folder.

    The console shows our messages from info up and only warnings and errors from libraries. The file
    also keeps our debug messages and the libraries' info messages, and a resumed run adds to it.
    """
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "mission.log"
    console = _Console()
    console.setFormatter(_ConsoleFormat())
    console.addFilter(lambda r: r.levelno >= (logging.INFO if r.name.startswith("bv_autonomy") else logging.WARNING))
    file = logging.FileHandler(path, mode="a", encoding="utf-8")
    file.setFormatter(_FileFormat("%(asctime)s.%(msecs)03d %(levelname)-7s %(source)-13s %(message)s",
                                  "%Y-%m-%d %H:%M:%S"))
    file.addFilter(lambda r: r.levelno >= (logging.DEBUG if r.name.startswith("bv_autonomy") else logging.INFO))
    root = logging.getLogger()
    for handler in _handlers:
        root.removeHandler(handler)
        handler.close()
    _handlers[:] = [console, file]
    for handler in _handlers:
        root.addHandler(handler)
    root.setLevel(logging.DEBUG)
    return path


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
        except BaseException as error:
            if isinstance(error, Exception):
                log.exception("mission failed")
            else:
                log.warning("mission stopped")
            await self.on_abort()
            raise

    @abstractmethod
    async def run(self):
        """Flies the competition from start to finish."""

    async def on_abort(self):
        """Gets the aircraft somewhere safe after the mission fails or is stopped."""
