from abc import ABC, abstractmethod


class Mission(ABC):
    """One competition. Subclasses build their components in __init__ and write run()."""

    async def start(self):
        """Every mission inherits this. Don't override it."""
        try:
            await self.run()
        except Exception:
            await self.on_abort()
            raise

    @abstractmethod
    async def run(self):
        """The competition, step by step."""

    async def on_abort(self):
        """Called when run() fails. Override it to get the aircraft somewhere safe."""
