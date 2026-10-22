from bv_autonomy.missions.suas2026.suas2026 import Site
from bv_autonomy.types import Waypoint, WaypointSequence

# bv_bevy's suas_2026 world, from bv_core's sim_params.yaml. Official lap points are released at check-in.
SIM = Site(
    laps=WaypointSequence([
        Waypoint(36.21531017, -96.00994545, 60.96),
        Waypoint(36.21362892, -96.00981991, 60.96),
        Waypoint(36.21359247, -96.00813694, 60.96),
        Waypoint(36.21142292, -96.00821104, 60.96),
        Waypoint(36.21146430, -96.00483924, 60.96),
        Waypoint(36.21645553, -96.00483924, 60.96),
        Waypoint(36.21883578, -96.00271530, 60.96),
        Waypoint(36.21961682, -96.00447978, 60.96),
        Waypoint(36.21998623, -96.00686465, 60.96),
        Waypoint(36.21980137, -96.00921336, 60.96),
        Waypoint(36.21911447, -96.01129465, 60.96),
        Waypoint(36.21808622, -96.01282681, 60.96),
    ]),
    scan_area=WaypointSequence([
        Waypoint(36.216341, -96.010424, 0.0),
        Waypoint(36.216750, -96.007550, 0.0),
        Waypoint(36.218054, -96.007835, 0.0),
        Waypoint(36.217645, -96.010709, 0.0),
    ]),
)
