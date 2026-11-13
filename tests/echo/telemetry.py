"""Prints PX4's telemetry as it arrives, like ros2 topic echo. Stop it with Ctrl-C.

    .venv/bin/python tests/echo/telemetry.py --list
    .venv/bin/python tests/echo/telemetry.py position gps_info
    .venv/bin/python tests/echo/telemetry.py battery --platform sim
"""

import argparse
import asyncio
import time

from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
from mavsdk.asyncio.plugins.telemetry import TelemetryAsync

from bv_autonomy.components.platform import CompDronePlatform, SimPlatform, X650Platform

PLATFORMS = {"comp_drone": CompDronePlatform, "x650": X650Platform, "sim": SimPlatform}
TOPICS = sorted(name.removeprefix("subscribe_") for name in dir(TelemetryAsync) if name.startswith("subscribe_"))


async def echo(address: str, topics: list[str], timeout: float) -> None:
    """Connects to PX4 without waiting for a fix and prints every message on the given topics."""
    mavsdk = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
    await mavsdk.add_any_connection(address)
    system = await mavsdk.first_autopilot(timeout_s=timeout)
    if system is None:
        raise SystemExit(f"no autopilot found at {address} within {timeout:.0f} s")
    telemetry = TelemetryAsync(system)
    print(f"connected to {address}, echoing {', '.join(topics)}", flush=True)
    try:
        await asyncio.gather(*(print_stream(topic, getattr(telemetry, f"subscribe_{topic}")()) for topic in topics))
    finally:
        telemetry.destroy()
        mavsdk.destroy()


async def print_stream(topic: str, stream) -> None:
    async for value in stream:
        print(f"{time.strftime('%H:%M:%S')} {topic}: {value}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="Print PX4 telemetry topics as they arrive.")
    parser.add_argument("topics", nargs="*", choices=TOPICS, metavar="topic", help="topics to echo; see --list")
    parser.add_argument("--list", action="store_true", help="list the topics and exit")
    parser.add_argument("--platform", choices=list(PLATFORMS), default="comp_drone",
                        help="which aircraft to connect to, or sim for bv_bevy's simulator")
    parser.add_argument("--timeout", type=float, default=30.0, help="seconds to wait for PX4")
    args = parser.parse_args()
    if args.list:
        print("\n".join(TOPICS))
        return
    if not args.topics:
        parser.error("name at least one topic, or --list to see them")
    address = PLATFORMS[args.platform]().drone._address
    try:
        asyncio.run(echo(address, args.topics, args.timeout))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
