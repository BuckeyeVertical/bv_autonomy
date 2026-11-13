async def test_pixhawk_reports_telemetry(drone):
    """Connecting already waits for a position fix, a home position and telemetry; this shows what PX4 reports."""
    position, home = drone.position(), drone.home()
    print(f"position {position.lat:.6f}, {position.lon:.6f}, {position.alt:.1f} m")
    print(f"home {home.lat:.6f}, {home.lon:.6f}")
    print(f"heading {drone.heading():.0f} deg, battery {drone.battery():.0%}, "
          f"armed {drone.is_armed()}, in air {drone.in_air()}")
    assert 0.0 <= drone.battery() <= 1.0
