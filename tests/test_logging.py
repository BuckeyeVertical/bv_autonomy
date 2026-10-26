import logging

import pytest
from mavsdk import LogLevel

from bv_autonomy.components.drone import mavsdk_drone
from bv_autonomy.missions.base import start_logging


@pytest.fixture
def logs(tmp_path):
    """Starts mission logging into a temp folder and returns the log file."""
    return start_logging(tmp_path / "logs")


def read(path):
    """Returns the log file's contents once everything logged so far is written."""
    for handler in logging.getLogger().handlers:
        handler.flush()
    return path.read_text()


def test_our_progress_goes_to_the_console_and_the_file(logs, capsys):
    logging.getLogger("bv_autonomy.missions.suas2026.suas2026").info("confirmed person at 36.2, -96.0")
    console = capsys.readouterr().err
    assert console.strip().endswith("confirmed person at 36.2, -96.0")
    assert "INFO" not in console and "bv_autonomy" not in console
    line = next(line for line in read(logs).splitlines() if "confirmed person at 36.2, -96.0" in line)
    assert "INFO" in line and "suas2026" in line


def test_our_detail_goes_only_to_the_file(logs, capsys):
    logging.getLogger("bv_autonomy.missions.suas2026.suas2026").debug("window person x.x..")
    assert capsys.readouterr().err == ""
    assert "window person x.x.." in read(logs)


def test_library_chatter_stays_out_of_the_console_but_warnings_show(logs, capsys):
    library = logging.getLogger("mavsdk")
    library.debug("Current: 2, total: 3")
    library.info("New system on: 127.0.0.1:14580")
    library.warning("Received ack for not-existing command: 176!")
    console = capsys.readouterr().err
    assert console.strip().endswith("WARNING mavsdk: Received ack for not-existing command: 176!")
    assert "New system" not in console
    written = read(logs)
    assert "New system on" in written and "Current: 2" not in written


def test_a_second_start_appends_and_replaces_only_its_own_handlers(logs, capsys):
    logging.getLogger("bv_autonomy.test").info("first run")
    other = logging.NullHandler()
    logging.getLogger().addHandler(other)
    try:
        start_logging(logs.parent)
        logging.getLogger("bv_autonomy.test").info("resumed run")
        assert other in logging.getLogger().handlers
        assert sum(getattr(h, "baseFilename", None) == str(logs) for h in logging.getLogger().handlers) == 1
        written = read(logs)
        assert written.index("first run") < written.index("resumed run")
    finally:
        logging.getLogger().removeHandler(other)


def test_mavsdk_messages_are_passed_to_python_logging(caplog):
    with caplog.at_level(logging.DEBUG, logger="mavsdk"):
        assert mavsdk_drone._forward_mavsdk_log(LogLevel.WARN, "link lost", None, 0) is True
    assert [(r.name, r.levelno, r.getMessage()) for r in caplog.records] == [("mavsdk", logging.WARNING, "link lost")]
