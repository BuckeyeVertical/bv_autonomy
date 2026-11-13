import argparse
import asyncio
import threading

from bv_autonomy import paths
from bv_autonomy.components.platform import X650Platform
from bv_autonomy.missions.base import start_logging
from bv_autonomy.missions.suas2026 import MissionState, Suas2026, sites


async def confirm(prompt: str) -> bool:
    """Asks the operator to type y, keeping telemetry and the dashboard running and letting Ctrl-C exit at once."""
    loop = asyncio.get_running_loop()
    answer = loop.create_future()

    def ask():
        try:
            reply = input(prompt)
        except EOFError:
            reply = ""
        loop.call_soon_threadsafe(lambda: answer.done() or answer.set_result(reply))

    threading.Thread(target=ask, daemon=True).start()
    return (await answer).strip().lower() == "y"


async def main(resume: bool):
    if not resume:
        paths.archive_outputs(paths.SUAS2026_OUTPUT)
    start_logging(paths.SUAS2026_OUTPUT / "logs")
    platform = X650Platform()
    site = sites.GLACIER
    state = MissionState.load() if resume else MissionState.new(site, platform.intrinsics)
    state.save()
    mission = Suas2026(site, state, platform)
    try:
        await mission.connect(resuming=resume)
        if not await confirm("Preflight checks passed. Start the mission? [y/N] "):
            return
        await mission.start()
    finally:
        await mission.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--resume", action="store_true", help="continue from where the last run left off")
    args = parser.parse_args()
    asyncio.run(main(args.resume))
