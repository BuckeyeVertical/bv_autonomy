import argparse
import asyncio

from bv_autonomy.missions.suas2026 import MissionState, Suas2026
from bv_autonomy.missions.suas2026.sites import SIM


async def main(resume: bool):
    state = MissionState.load() if resume else MissionState.new(SIM)
    state.save()                 # so a new run never resumes an older run's file
    mission = Suas2026(SIM, state)
    await mission.connect(resuming=resume)
    await mission.start()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--resume", action="store_true", help="continue from where the last run left off")
    asyncio.run(main(parser.parse_args().resume))
