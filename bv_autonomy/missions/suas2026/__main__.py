import asyncio

from bv_autonomy.missions.suas2026 import Suas2026
from bv_autonomy.missions.suas2026.sites import SIM


async def main():
    mission = Suas2026(SIM)
    await mission.connect()
    await mission.start()


if __name__ == "__main__":
    asyncio.run(main())
