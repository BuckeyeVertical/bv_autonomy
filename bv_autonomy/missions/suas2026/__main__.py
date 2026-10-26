import argparse
import asyncio
import logging

from bv_autonomy import paths
from bv_autonomy.components.platform import SimPlatform
from bv_autonomy.missions.suas2026 import MissionState, Suas2026
from bv_autonomy.missions.suas2026.sites import SIM


async def main(resume: bool):
    if not resume:
        paths.archive_outputs(paths.SUAS2026_OUTPUT)
    platform = SimPlatform()
    state = MissionState.load() if resume else MissionState.new(SIM, platform.intrinsics)
    state.save()
    mission = Suas2026(SIM, state, platform)
    await mission.connect(resuming=resume)
    await mission.start()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--resume", action="store_true", help="continue from where the last run left off")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    asyncio.run(main(args.resume))
