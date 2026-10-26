import json

import pytest

from bv_autonomy.missions.suas2026 import MissionState
from bv_autonomy.missions.suas2026.confirmation import Sighting
from bv_autonomy.missions.suas2026.suas2026 import LAP_COUNT
from bv_autonomy.types import Waypoint, WaypointSequence


def test_new_state_repeats_laps_and_closes_route(site):
    route = MissionState.new(site).lap_route
    assert isinstance(route, WaypointSequence)
    assert route == site.laps * LAP_COUNT + [site.laps[0]]


def test_save_load_preserves_waypoints_and_order(site):
    state = MissionState.new(site)
    state.targets.append(Sighting("tent", Waypoint(36.1, -96.1, 0.0)))
    state.rejected.append(Sighting("person", Waypoint(36.2, -96.2, 0.0)))
    state.delivered.append(Sighting("person", Waypoint(36.3, -96.3, 0.0)))
    state.save()
    restored = MissionState.load()
    assert restored == state
    assert restored is not state
    assert isinstance(restored.lap_route, WaypointSequence)
    assert all(isinstance(point, Waypoint) for point in restored.lap_route)


def test_save_replaces_previous_state_without_temp_file(site):
    state = MissionState.new(site)
    state.save()
    state.lap_route = WaypointSequence(state.lap_route[2:])
    state.save()
    assert MissionState.load() == state
    assert not state.path.with_suffix(".tmp").exists()


def test_load_missing_file_raises():
    with pytest.raises(FileNotFoundError):
        MissionState.load()


def test_load_malformed_json_raises():
    MissionState.path.parent.mkdir(parents=True)
    MissionState.path.write_text("{")
    with pytest.raises(json.JSONDecodeError):
        MissionState.load()


def test_failed_atomic_replace_keeps_previous_state(site, monkeypatch):
    from bv_autonomy.missions import base

    old = MissionState.new(site)
    old.save()

    def fail_replace(*args):
        raise OSError("disk failure")

    monkeypatch.setattr(base.os, "replace", fail_replace)
    with pytest.raises(OSError, match="disk failure"):
        MissionState(lap_route=WaypointSequence(), scan_route=WaypointSequence()).save()
    assert MissionState.load() == old
