"""An example study plugin (examples/studies/boat_cards.yaml names this dir under plugins:): a boat mission with a
speed cap, as Python mission code. A study refers to it as ``mission: capped:CappedAdapter``; crane puts this dir on
sys.path for that study's runs only and runs the arm on the scalar path (Python mission code)."""

from domains.boat.adapter import BoatAdapter, Mission


class CappedMission(Mission):
    """The reference mission, never asking for more than ``cap`` m/s."""

    cap = None

    def tick(self, obs):
        self.ctx.navigator.speed_limit(self.cap)
        return super().tick(obs)


class CappedAdapter(BoatAdapter):
    """BoatAdapter whose controller is CappedMission. Option ``cap_mps`` (recorded in the run identity)."""

    def __init__(self, base=None, cap_mps=1.0, **options):
        super().__init__(base, **options)
        self.cap = float(cap_mps)
        self.ident['mission'] = f'capped/{self.cap:g}'

    def controller(self, ctx):
        self.current = CappedMission(ctx, self.plan(ctx.scenario))
        self.current.cap = self.cap
        return self.current
