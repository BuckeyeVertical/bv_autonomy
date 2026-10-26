import pytest

from bv_autonomy.missions.suas2026.confirmation import ConfirmationWindow, Sighting, wanted
from bv_autonomy.types import Waypoint

HERE = Waypoint(36.0, -96.0, 0.0)
M = 1 / 111_320          # about one meter of latitude, in degrees


def near(meters_north: float) -> Waypoint:
    return Waypoint(HERE.lat + meters_north * M, HERE.lon, 0.0)


@pytest.fixture
def window():
    return ConfirmationWindow({"person": 0, "tent": 1}, hits=2, frames=5, radius_m=11.0)


def test_two_hits_in_window_confirm_at_centroid_even_with_gaps(window):
    assert window.add([Sighting("person", near(0))]) is None
    assert window.add([]) is None
    assert window.add([]) is None
    confirmed = window.add([Sighting("person", near(4))])
    assert confirmed.label == "person"
    assert confirmed.position.lat == pytest.approx(near(2).lat)


def test_sightings_too_far_apart_do_not_confirm(window):
    window.add([Sighting("person", near(0))])
    assert window.add([Sighting("person", near(20))]) is None


def test_a_hit_older_than_the_window_no_longer_counts(window):
    window.add([Sighting("person", near(0))])
    for _ in range(4):
        window.add([])
    assert window.add([Sighting("person", near(0))]) is None      # the first hit just left the window
    assert window.add([Sighting("person", near(0))]).label == "person"


def test_labels_are_confirmed_separately(window):
    window.add([Sighting("person", near(0)), Sighting("tent", near(100))])
    assert window.add([Sighting("tent", near(101))]).label == "tent"
    assert window.add([Sighting("person", near(1))]).label == "person"


def test_clear_forgets_earlier_hits(window):
    window.add([Sighting("person", near(0))])
    window.clear()
    assert window.add([Sighting("person", near(0))]) is None
    assert window.summary(set())["frames"] == 1


def test_first_sighting_per_frame_is_used(window):
    window.add([Sighting("person", near(0)), Sighting("person", near(50))])
    assert window.add([Sighting("person", near(5))]).label == "person"


def test_summary_matches_dashboard_format(window):
    window.add([Sighting("person", near(0))])
    window.add([])
    assert window.summary({"tent"}) == {
        "required": 2, "window": 5, "frames": 2,
        "classes": [
            {"class_id": 0, "name": "person", "confirmed": False, "hits": [True, False]},
            {"class_id": 1, "name": "tent", "confirmed": True, "hits": [False, False]},
        ],
    }


@pytest.mark.parametrize("hits,frames", [(0, 5), (6, 5)])
def test_impossible_rule_rejected(hits, frames):
    with pytest.raises(ValueError, match="hits"):
        ConfirmationWindow({"person": 0}, hits, frames, 11.0)


def test_wanted_skips_approved_labels_and_rejected_spots():
    rejected = [Sighting("person", HERE)]
    assert not wanted(Sighting("person", near(5)), [], rejected, 11.0)
    assert wanted(Sighting("person", near(30)), [], rejected, 11.0)
    assert wanted(Sighting("tent", HERE), [], rejected, 11.0)
    assert not wanted(Sighting("tent", HERE), [Sighting("tent", near(500))], rejected, 11.0)
