# SPDX-License-Identifier: GPL-3.0-or-later
"""The experiment-2 stacks. All share the estimator, sensors, primitive set, actuation limits, the stopping rule
(StopRule: p99-age tau + simulated lag/jerk braking) and the arbiter's stall watchdog (episode loop). They differ
only in what they plan on and how they adapt:

  REACTIVE      current-frame map only (newest stereo + newest LiDAR frame, rebuilt every cycle), no prior, no
                memory, 8 m horizon, goal-heading score.
  MAP_RHP       accumulated map + prior map at full trust (w_prior = 1), 14 m horizon, cost-to-go score.
  HYBRID_CONF   graded: w_prior = c_align, memory horizon from c_loc, planning horizon from c_obs, speed cap from
                min(c_obs, c_loc), extra margin from (1 - c_loc).
  HYBRID_FIXED  B7c: one scalar (signal 'conf' = min(c_align, c_obs, c_loc), or 'agree' = map agreement c_agree),
                one threshold theta (+0.05 hysteresis): MAP_RHP mode
                above, REACTIVE-like fallback (0.3 s memory, no prior, 8 m, v_lim * v_fb) below.
  OPT_BACKUP    B7b (FASTER/SUPER-style): primitives scored on an optimistic belief (prior at full trust,
                unknown = free) but a command is feasible only if braking along it stops inside space the fresh map
                has observed free (prior and unknown excluded): a known-free backup stop every cycle.
  MAP_RHP_N1    reference only: MAP_RHP planner with the N1 safety layer (tau = 0.10 s, analytic v^2/2a braking,
                no watchdog). Shows what the safety-layer fixes buy on these scenarios.
"""
import math

from .core import Belief, StopRule, evaluate

REACTIVE, MAP_RHP, HYBRID_CONF, HYBRID_FIXED, MAP_RHP_N1 = "reactive", "map_rhp", "hybrid_conf", "hybrid_fixed", "map_rhp_n1"
OPT_BACKUP = "opt_backup"
MAIN = [REACTIVE, MAP_RHP, OPT_BACKUP, HYBRID_CONF, HYBRID_FIXED]

CONF_DEFAULT = {"s_align": 1.0, "a0": 0.35, "a1": 0.65, "o0": 0.4, "k_loc": 0.15, "H_min": 8.0,
                "v_floor": 0.35, "mem_min": 0.5, "m_loc": 0.4, "age_growth": 0.05}
FIXED_DEFAULT = {"signal": "conf", "theta": 0.5, "v_fb": 0.6, "s_align": 1.0, "a0": 0.35, "a1": 0.65, "o0": 0.4, "k_loc": 0.15,
                 "age_growth": 0.05}
H_MAX = 14.0
H_REACT = 8.0
CTG_PERIOD = 0.5


def _clip01(v):
    return max(0.0, min(1.0, v))


def confidences(sig, p):
    """sig: dict(sigma_align_rep, rep_age, agree, q_obs, sigma_loc_rate) -> (c_align, c_obs, c_loc)."""
    s_eff = sig["sigma_align_rep"] + p["age_growth"] * sig["rep_age"]
    c_rep = _clip01(1.0 - s_eff / p["s_align"])
    c_agree = _clip01((sig["agree"] - p["a0"]) / max(p["a1"] - p["a0"], 1e-6))
    c_align = min(c_rep, c_agree)
    c_obs = _clip01((sig["q_obs"] - p["o0"]) / max(1.0 - p["o0"], 1e-6))
    c_loc = _clip01(1.0 - (sig["sigma_loc_rate"] - 0.03) / p["k_loc"])
    return c_align, c_obs, c_loc


class N1Rule(StopRule):
    """N1 baseline rule: tau = 0.10 s, analytic braking at 0.8 x tilt-limited deceleration, no lag/jerk term."""

    def __init__(self, veh, margin=0.5):
        super().__init__(veh, tau_prior=0.10, margin=margin)
        self.a = 0.8 * min(veh["a_h_tilt"], veh["a_h_max"])

    def observe_age(self, age):
        pass

    def dist(self, v, extra_margin=0.0):
        return v * 0.10 + v * v / (2.0 * self.a) + self.margin + extra_margin


class Stack:
    def __init__(self, name, veh, params=None):
        self.name, self.veh = name, veh
        self.p = dict(CONF_DEFAULT if name == HYBRID_CONF else FIXED_DEFAULT if name == HYBRID_FIXED else {})
        self.p.update(params or {})
        self.rule = N1Rule(veh) if name == MAP_RHP_N1 else StopRule(veh)
        self.watchdog = name != MAP_RHP_N1
        self.uses_prior = name != REACTIVE
        self.uses_eph = name == REACTIVE
        self.ctg, self.ctg_t = None, -1e9
        self.mode = "map"
        self.mode_switches = 0
        self.fallback_cycles = 0
        self.trace = {"w_prior": 0.0, "v_cap": 0.0, "H": 0.0, "n": 0}

    def cycle(self, t, x, y, vx, vy, goal, v_lim, acc, eph, prior, shift, sig, v_cap_ext=math.inf):
        """Returns the evaluate() dict plus 'fallback' (bool). prior = (L, obs) or None."""
        from .core import cost_to_go
        pL, pO = prior if prior is not None else (None, None)
        name = self.name
        extra_margin = 0.0
        use_ctg = True
        if name == REACTIVE:
            b = Belief(eph=eph, t=t)
            H, v_cap, use_ctg, w = H_REACT, v_lim, False, 0.0
        elif name == OPT_BACKUP:
            w = 1.0
            b = Belief(acc=acc, prior_L=pL, prior_obs=pO, w_prior=w, shift=shift, t=t)
            H, v_cap = H_MAX, v_lim
        elif name in (MAP_RHP, MAP_RHP_N1):
            w = 1.0
            b = Belief(acc=acc, prior_L=pL, prior_obs=pO, w_prior=w, shift=shift, t=t)
            H, v_cap = H_MAX, v_lim
        elif name == HYBRID_CONF:
            ca, co, cl = confidences(sig, self.p)
            w = ca
            mem = self.p["mem_min"] + (30.0 - self.p["mem_min"]) * cl
            b = Belief(acc=acc, mem_s=mem, prior_L=pL, prior_obs=pO, w_prior=w, shift=shift, t=t)
            H = self.p["H_min"] + (H_MAX - self.p["H_min"]) * co
            v_cap = v_lim * (self.p["v_floor"] + (1.0 - self.p["v_floor"]) * min(co, cl))
            extra_margin = self.p["m_loc"] * (1.0 - cl)
            fb = w < 0.5 or min(co, cl) < 0.5
            if fb != (self.mode == "fallback"):
                if fb:
                    self.mode_switches += 1
                self.mode = "fallback" if fb else "map"
        elif name == HYBRID_FIXED:
            cs = confidences(sig, self.p)
            if self.p["signal"] == "agree":
                c = _clip01((sig["agree"] - self.p["a0"]) / max(self.p["a1"] - self.p["a0"], 1e-6))
            else:
                c = min(cs)
            th = self.p["theta"]
            if self.mode == "map" and c < th - 0.025:
                self.mode, self.mode_switches = "fallback", self.mode_switches + 1
            elif self.mode == "fallback" and c > th + 0.025:
                self.mode = "map"
            if self.mode == "map":
                w = 1.0
                b = Belief(acc=acc, prior_L=pL, prior_obs=pO, w_prior=w, shift=shift, t=t)
                H, v_cap = H_MAX, v_lim
            else:
                w = 0.0
                b = Belief(acc=acc, mem_s=0.3, t=t)
                H, v_cap, use_ctg = H_REACT, v_lim * self.p["v_fb"], False
        else:
            raise ValueError(name)
        if self.mode == "fallback":
            self.fallback_cycles += 1
        v_cap = min(v_cap, v_cap_ext)
        ctg = None
        if use_ctg:
            if self.ctg is None or t - self.ctg_t >= CTG_PERIOD or abs(w - getattr(self, "_ctg_w", w)) > 0.25:
                self.ctg, self.ctg_t, self._ctg_w = cost_to_go(b, acc, goal[0]), t, w
            ctg = self.ctg
        if name == OPT_BACKUP:
            out = evaluate(b, acc, x, y, vx, vy, goal, v_cap, self.veh, self.rule, horizon=H, ctg=ctg,
                           optimistic=True, b_safe=Belief(acc=acc, t=t))
        else:
            out = evaluate(b, acc, x, y, vx, vy, goal, v_cap, self.veh, self.rule, horizon=H, ctg=ctg,
                           extra_margin=extra_margin)
        out["fallback"] = self.mode == "fallback"
        tr = self.trace
        tr["w_prior"] += w
        tr["v_cap"] += v_cap
        tr["H"] += H
        tr["n"] += 1
        return out
