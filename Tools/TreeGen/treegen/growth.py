# SPDX-License-Identifier: GPL-3.0-or-later
"""Self-organizing tree growth (Palubicki, Horel, Longay, Runions, Lane, Mech, Prusinkiewicz 2009,
"Self-organizing tree models for image synthesis", ACM TOG 28(3)). Implemented from the paper; no code
from any SOTM implementation was used.

One iteration = one season:
  1. light: shared shadow-voxel grid (light.ShadowGrid) and/or space-colonization markers
  2. basipetal pass: Q accumulates from buds to the root
  3. acropetal pass: Borchert-Honda allocation v_m = v*lQm/(lQm+(1-l)Ql), generalised to n sinks
  4. bud fates: floor(v) metamers, dormancy -> latent, abortion, sympodial terminal abortion,
     epicormic regeneration
  5. direction: d = norm(w0 d_prev + wV V + wg(order) up + wh(order) horiz + wn noise)
  6. shedding: lateral subtrees with Q/metamer < theta after a grace period; optional dead stubs that
     decay after a drawn time
  7. radii: pipe model r_p^n = sum r_c^n (+ shed contributions, + r_tip^n at tips), n per species

Everything is vectorised across buds/nodes; iteration order is node-id order, all RNG draws come from
the tree's own "growth" stream, so results are bit-reproducible for a given (forest_seed, tree_id).
"""
from __future__ import annotations

import numpy as np

from .light import ShadowGrid
from .rng import stream
from .skeleton import Skeleton, params_hash
from . import spacecol

UP = np.array([0.0, 0.0, 1.0])

TERMINAL, AXILLARY = 0, 1
ACTIVE, LATENT, DEAD = 0, 1, 2


def _norm(v):
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, 1e-12)


def _by_order(lst, order):
    a = np.asarray(lst, dtype=np.float64)
    return a[np.minimum(order, len(a) - 1)]


class TreeGrower:
    NODE_FIELDS = {
        "pos": (3, np.float64), "dir": (3, np.float64), "nrm": (3, np.float64),
        "parent": (None, np.int64), "order": (None, np.int64), "alive": (None, bool),
        "is_main": (None, bool), "birth": (None, np.int64), "dead_since": (None, np.int64),
        "decay_at": (None, np.int64), "radius": (None, np.float64), "shed_pipe": (None, np.float64),
        "light": (None, np.float64), "phyl": (None, np.float64), "depth": (None, np.int64),
    }
    BUD_FIELDS = {
        "b_node": (None, np.int64), "b_kind": (None, np.int64), "b_dir": (3, np.float64),
        "b_state": (None, np.int64), "b_age": (None, np.int64), "b_order": (None, np.int64),
    }

    def __init__(self, species: dict, tree_id: int, forest_seed: int, origin=(0.0, 0.0, 0.0),
                 start_season: int = 0, seasons: int | None = None):
        self.sp = species
        self.tree_id = int(tree_id)
        self.forest_seed = int(forest_seed)
        self.origin = np.asarray(origin, dtype=np.float64)
        self.start_season = int(start_season)
        self.seasons = int(seasons if seasons is not None else species["seasons"])
        self.rng = stream(forest_seed, tree_id, "growth")
        self.n = 0
        self.nb = 0
        self._cap = 0
        self._bcap = 0
        self._grow_nodes(256)
        self._grow_buds(256)
        sp = species
        # root node
        self._add_nodes(pos=np.zeros((1, 3)), dir=UP[None], nrm=np.array([[1.0, 0, 0]]),
                        parent=np.array([-1]), order=np.array([0]), is_main=np.array([False]),
                        birth=np.array([self.start_season]), phyl=np.array([0.0]))
        self.radius[0] = sp["r_tip"]
        tilt = self.rng.normal(0, np.radians(sp.get("initial_tilt_deg", 3.0)), 2)
        d0 = _norm(np.array([tilt[0], tilt[1], 1.0]))
        self._add_buds(np.array([0]), np.array([TERMINAL]), d0[None], np.array([0]))
        self.markers = np.zeros((0, 3))
        env = sp.get("environment", "shadow")
        if env in ("space", "both"):
            sc = sp["space"]
            mr = stream(forest_seed, tree_id, "markers")
            self.markers = spacecol.crown_markers(mr, sc.get("shape", "cone"), sc["height"], sc["base"],
                                                  sc["radius"], int(sc["n_markers"]))
        self.season_log = []

    # ------------------------------------------------------------------ storage
    def _grow_nodes(self, need):
        if need <= self._cap:
            return
        cap = max(need, self._cap * 2, 256)
        for name, (w, dt) in self.NODE_FIELDS.items():
            shape = (cap, w) if w else (cap,)
            new = np.zeros(shape, dtype=dt)
            if self._cap:
                new[: self.n] = getattr(self, name)[: self.n]
            setattr(self, name, new)
        self._cap = cap

    def _grow_buds(self, need):
        if need <= self._bcap:
            return
        cap = max(need, self._bcap * 2, 256)
        for name, (w, dt) in self.BUD_FIELDS.items():
            shape = (cap, w) if w else (cap,)
            new = np.zeros(shape, dtype=dt)
            if self._bcap:
                new[: self.nb] = getattr(self, name)[: self.nb]
            setattr(self, name, new)
        self._bcap = cap

    def _add_nodes(self, pos, dir, nrm, parent, order, is_main, birth, phyl):
        k = len(pos)
        self._grow_nodes(self.n + k)
        s = slice(self.n, self.n + k)
        self.pos[s] = pos
        self.dir[s] = dir
        self.nrm[s] = nrm
        self.parent[s] = parent
        self.order[s] = order
        self.alive[s] = True
        self.is_main[s] = is_main
        self.birth[s] = birth
        self.dead_since[s] = -1
        self.decay_at[s] = -1
        self.radius[s] = self.sp["r_tip"]
        self.shed_pipe[s] = 0.0
        self.light[s] = 1.0
        self.phyl[s] = phyl
        self.depth[s] = np.where(parent >= 0, self.depth[np.maximum(parent, 0)] + 1, 0)
        ids = np.arange(self.n, self.n + k)
        self.n += k
        return ids

    def _add_buds(self, node, kind, d, order):
        k = len(node)
        if k == 0:
            return
        self._grow_buds(self.nb + k)
        s = slice(self.nb, self.nb + k)
        self.b_node[s] = node
        self.b_kind[s] = kind
        self.b_dir[s] = _norm(d)
        self.b_state[s] = ACTIVE
        self.b_age[s] = 0
        self.b_order[s] = order
        self.nb += k

    # ------------------------------------------------------------------ tree reductions
    def _levels(self):
        n = self.n
        dep = self.depth[:n]
        order = np.argsort(dep, kind="stable")
        bounds = np.searchsorted(dep[order], np.arange(dep.max() + 2))
        return order, bounds

    def _accumulate_up(self, vals, levels):
        """Subtree sums: out[i] = vals[i] + sum over children out[c]."""
        out = vals.astype(np.float64).copy()
        order, bounds = levels
        par = self.parent[: self.n]
        for d in range(len(bounds) - 2, 0, -1):
            ids = order[bounds[d]:bounds[d + 1]]
            if len(ids):
                np.add.at(out, par[ids], out[ids])
        return out

    def active_bud_world_positions(self):
        if self.nb == 0:
            return np.zeros((0, 3))
        act = self.b_state[: self.nb] == ACTIVE
        return self.origin + self.pos[self.b_node[: self.nb][act]]

    # ------------------------------------------------------------------ one season
    def step(self, season: int, grid: ShadowGrid | None):
        sp = self.sp
        rng = self.rng
        n = self.n
        lam = float(sp["lambda"])
        nb = self.nb
        bn = self.b_node[:nb]
        bstate = self.b_state[:nb]
        act = np.nonzero(bstate == ACTIVE)[0]
        lat = np.nonzero(bstate == LATENT)[0]

        # ---- 1. environment
        P = self.pos[bn[act]]
        D = self.b_dir[act]
        env = sp.get("environment", "shadow")
        Q = np.ones(len(act))
        V = D.copy()
        if grid is not None and env in ("shadow", "both"):
            Q = grid.exposure(self.origin + P)
            V = grid.best_direction(self.origin + P, D)
        if env in ("space", "both") and len(self.markers):
            sc = sp["space"]
            cnt, Vm = spacecol.marker_environment(self.markers, P, D, sc["r_perc"],
                                                  np.cos(np.radians(sc.get("perc_angle_deg", 90.0))))
            Qm = np.minimum(cnt / float(sc.get("q_ref", 3.0)), 1.0)
            # buds outside the envelope keep a base exposure so the leader can climb into it
            Qm = np.maximum(Qm, float(sc.get("q_floor", 0.0)))
            # the conifer leader sits above the attractor envelope; never starve it of markers
            lead = (self.b_order[act] == 0) & (self.b_kind[act] == TERMINAL)
            Qm[lead] = np.maximum(Qm[lead], float(sc.get("leader_q_floor", 1.0)))
            Q = Q * Qm
            has = cnt > 0
            V[has] = _norm(V[has] + 2.0 * Vm[has])

        # ---- 2. basipetal accumulation
        levels = self._levels()
        Qn = np.bincount(bn[act], weights=Q, minlength=n)
        Qsub = self._accumulate_up(Qn, levels)
        alive = self.alive[:n]
        cnt_sub = self._accumulate_up(alive.astype(np.float64), levels)

        # ---- 3. acropetal Borchert-Honda allocation
        par = self.parent[:n]
        order = self.order[:n]
        decay = float(sp.get("order_vigor_decay", 1.0))
        kids = np.arange(1, n)
        gam = float(sp.get("favor_bright", 1.0))   # <1 flattens light-driven sibling rivalry
        wc = Qsub[kids] ** gam * np.where(self.is_main[kids], lam, 1.0 - lam) * decay ** order[kids]
        wb = Q ** gam * np.where(self.b_kind[act] == TERMINAL, lam, 1.0 - lam) * decay ** self.b_order[act]
        W = np.bincount(par[kids], weights=wc, minlength=n) + np.bincount(bn[act], weights=wb, minlength=n)
        v = np.zeros(n)
        v[0] = float(sp["alpha"]) * Qsub[0]
        frac = np.zeros(n)
        frac[kids] = np.where(W[par[kids]] > 0, wc / np.maximum(W[par[kids]], 1e-300), 0.0)
        o, bounds = levels
        for d in range(1, len(bounds) - 1):
            ids = o[bounds[d]:bounds[d + 1]]
            v[ids] = v[par[ids]] * frac[ids]
        vb = v[bn[act]] * np.where(W[bn[act]] > 0, wb / np.maximum(W[bn[act]], 1e-300), 0.0)
        # hydraulic height limitation toward the species' asymptotic height
        hmax = float(sp["height_max"])
        z = P[:, 2]
        vb *= np.clip(1.0 - (np.maximum(z, 0) / hmax) ** float(sp.get("height_exponent", 3.0)), 0.0, 1.0)

        # ---- 4. bud fates
        nmax = _by_order(sp.get("max_metamers_by_order", [sp["max_metamers"]]), self.b_order[act])
        nmet = np.minimum(np.floor(vb), nmax).astype(np.int64)
        lmin = int(sp.get("leader_min_metamers", 0))
        if lmin > 0:
            lead = (self.b_order[act] == 0) & (self.b_kind[act] == TERMINAL) & (vb > 0)
            nmet[lead] = np.maximum(nmet[lead], lmin)
        u_abort = rng.random(len(act))
        aborted = (u_abort < float(sp.get("p_abort", 0.0))) & (nmet < 1)
        grow = (nmet >= 1) & ~aborted
        dormant = ~grow & ~aborted
        self.b_state[act[aborted]] = DEAD
        self.b_age[act[dormant]] += 1
        to_latent = act[dormant][self.b_age[act[dormant]] > int(sp.get("dormancy_seasons", 3))]
        self.b_state[to_latent] = LATENT
        # epicormic regeneration of latent buds in good light
        if len(lat) and float(sp.get("p_regen", 0.0)) > 0:
            ql = grid.exposure(self.origin + self.pos[bn[lat]]) if grid is not None else np.ones(len(lat))
            u = rng.random(len(lat))
            regen = (u < float(sp["p_regen"])) & (ql > float(sp.get("regen_light", 0.6)))
            self.b_state[lat[regen]] = ACTIVE
            self.b_age[lat[regen]] = 0

        G = act[grow]
        if len(G):
            self._grow_shoots(season, G, nmet[grow], V[grow])
            self.b_state[G] = DEAD

        # ---- 6. shedding
        self._shed(season, Qsub, cnt_sub)
        # buds on dead wood (fresh stubs) never grow again
        nb = self.nb
        dead_host = ~self.alive[self.b_node[:nb]]
        self.b_state[:nb][dead_host] = DEAD
        self._compact()
        # ---- 7. pipe-model radii
        self._update_radii()
        if len(self.markers):
            sc = sp["space"]
            new = self.birth[: self.n] == season
            self.markers = spacecol.remove_occupied(self.markers, self.pos[: self.n][new], sc["r_occ"])
        self.season_log.append((season, self.n, int((self.b_state[: self.nb] == ACTIVE).sum())))

    # ------------------------------------------------------------------ shoots
    def _grow_shoots(self, season, G, nmet, V):
        sp = self.sp
        rng = self.rng
        kind = self.b_kind[G]
        bord = self.b_order[G]
        cur = self.b_node[G].copy()
        dprev = self.b_dir[G].copy()
        # frame for the first node: terminal shoots transport the parent normal; laterals point away
        # from the parent axis
        pdir = self.dir[cur]
        side = dprev - (dprev * pdir).sum(1, keepdims=True) * pdir
        nprev = np.where((kind == TERMINAL)[:, None], self.nrm[cur], _norm(side))
        L = float(sp["internode_length"]) * float(sp.get("internode_order_decay", 1.0)) ** bord
        L = L * np.exp(rng.normal(0, float(sp.get("internode_jitter", 0.1)), len(G)))
        wprev = float(sp["w_prev"])
        wl = float(sp["w_light"])
        wg = _by_order(sp["w_grav_by_order"], bord)
        wh = _by_order(sp["w_horiz_by_order"], bord)
        wn = float(sp["w_noise"])
        div = np.radians(float(sp.get("divergence_deg", 137.5)))
        phyl = self.phyl[cur] + rng.uniform(0, 2 * np.pi, len(G)) * (kind == AXILLARY)
        jmax = int(nmet.max())
        shoot_nodes = -np.ones((len(G), jmax), dtype=np.int64)
        for j in range(jmax):
            m = nmet > j
            idx = np.nonzero(m)[0]
            dp = dprev[idx]
            hz = dp.copy()
            hz[:, 2] = 0.0
            hzn = np.linalg.norm(hz, axis=1)
            hz = np.where(hzn[:, None] > 1e-6, hz / np.maximum(hzn, 1e-12)[:, None], 0.0)
            noise = rng.standard_normal((len(idx), 3))
            d = _norm(wprev * dp + wl * V[idx] + wg[idx, None] * UP + wh[idx, None] * hz + wn * noise)
            if j == 0:
                # first metamer of a lateral keeps most of the bud's emergence angle
                lat0 = kind[idx] == AXILLARY
                d[lat0] = _norm(0.75 * self.b_dir[G[idx[lat0]]] + 0.25 * d[lat0])
            npv = nprev[idx]
            npv = npv - (npv * d).sum(1, keepdims=True) * d
            bad = np.linalg.norm(npv, axis=1) < 1e-6
            if bad.any():
                alt = np.cross(d[bad], np.array([1.0, 0, 0]))
                npv[bad] = alt
            npv = _norm(npv)
            is_main = (kind[idx] == TERMINAL) | (j > 0)
            ph = phyl[idx] + div * (j + 1)
            ids = self._add_nodes(pos=self.pos[cur[idx]] + L[idx, None] * d, dir=d, nrm=npv,
                                  parent=cur[idx], order=bord[idx], is_main=is_main,
                                  birth=np.full(len(idx), season), phyl=ph)
            shoot_nodes[idx, j] = ids
            cur[idx] = ids
            dprev[idx] = d
            nprev[idx] = npv
        self._place_buds(shoot_nodes, nmet, bord)

    def _place_buds(self, shoot_nodes, nmet, bord):
        sp = self.sp
        rng = self.rng
        phyllo = sp.get("phyllotaxis", "spiral")
        ang = None
        jit = np.radians(float(sp.get("branch_angle_jitter_deg", 8.0)))
        planar_min = int(sp.get("planar_order_min", 99))
        rows, cols = np.nonzero(shoot_nodes >= 0)
        nodes = shoot_nodes[rows, cols]
        last = cols == (nmet[rows] - 1)
        o = bord[rows]
        # number of axillary buds per node
        if phyllo == "whorl":
            k_last = int(sp.get("whorl_buds", 5))
            k = np.where(last, k_last, (rng.random(len(nodes)) < float(sp.get("p_intermediate_bud", 0.3))).astype(int))
            k = np.where(o >= 1, np.where(last, int(sp.get("whorl_buds_lateral", 2)),
                                          (rng.random(len(nodes)) < float(sp.get("p_intermediate_bud_lateral", 0.5))).astype(int)), k)
        elif phyllo == "decussate":
            k = np.full(len(nodes), 2)
        else:
            k = np.full(len(nodes), int(sp.get("buds_per_node", 1)))
        k = np.where(o + 1 > int(sp.get("max_order", 4)), 0, k)
        rep = np.repeat(np.arange(len(nodes)), k)
        slot = np.arange(len(rep)) - np.repeat(np.cumsum(k) - k, k)
        nd = nodes[rep]
        d = self.dir[nd]
        nrm = self.nrm[nd]
        bin_ = np.cross(d, nrm)
        kk = k[rep]
        if phyllo == "whorl":
            az = self.phyl[nd] + 2 * np.pi * slot / np.maximum(kk, 1) + rng.normal(0, 0.25, len(rep))
        elif phyllo == "decussate":
            az = self.phyl[nd] + np.pi * slot
        else:
            az = self.phyl[nd] + 2 * np.pi * slot / np.maximum(kk, 1)
        side = np.cos(az)[:, None] * nrm + np.sin(az)[:, None] * bin_
        planar = (o[rep] + 1) >= planar_min
        if planar.any():
            # flattened sprays: buds go left/right in the horizontal plane of the parent shoot
            lr = np.cross(d[planar], UP)
            lrn = np.linalg.norm(lr, axis=1, keepdims=True)
            lr = np.where(lrn > 1e-6, lr / np.maximum(lrn, 1e-12), side[planar])
            sgn = np.where((slot[planar] + nd[planar]) % 2 == 0, 1.0, -1.0)
            side[planar] = lr * sgn[:, None]
        ang = _by_order(sp["branch_angle_deg_by_order"], o + 1)
        th = np.radians(ang[rep]) + rng.normal(0, jit, len(rep))
        bd = np.cos(th)[:, None] * d + np.sin(th)[:, None] * side
        self._add_buds(nd, np.full(len(rep), AXILLARY), bd, o[rep] + 1)
        # terminal buds at shoot ends (sympodial species abort some)
        endn = nodes[last]
        keep = rng.random(len(endn)) >= float(sp.get("p_terminal_abort", 0.0))
        endn = endn[keep]
        self._add_buds(endn, np.full(len(endn), TERMINAL), self.dir[endn], self.order[endn])

    # ------------------------------------------------------------------ shedding
    def _shed(self, season, Qsub, cnt_sub):
        sp = self.sp
        rng = self.rng
        n0 = len(Qsub)          # nodes that existed before this season's growth
        n = self.n
        ids = np.arange(1, n0)
        eff = Qsub[ids] / np.maximum(cnt_sub[ids], 1.0)
        cand = ((~self.is_main[ids]) & self.alive[ids] &
                (season - self.birth[ids] >= int(sp.get("shed_grace", 3))) &
                (eff < float(sp["shed_threshold"])))
        roots = ids[cand]
        self.remove = np.zeros(n, dtype=bool)
        if len(roots):
            shed = np.zeros(n, dtype=bool)
            shed[roots] = True
            par = self.parent[:n]
            for i in range(1, n):     # propagate in topological order
                if not shed[i] and shed[par[i]]:
                    shed[i] = True
            # drop subtrees nested in another shed subtree
            roots = roots[~shed[par[roots]]]
            # dead stubs: keep the first stub_nodes nodes of the shed axis
            u = rng.random(len(roots))
            mk_stub = u < float(sp.get("p_stub", 0.0))
            stub = np.zeros(n, dtype=bool)
            nstub = int(sp.get("stub_nodes", 2))
            mc = -np.ones(n, dtype=np.int64)
            mm = np.nonzero(self.is_main[:n] & (par >= 0))[0]
            mc[par[mm[::-1]]] = mm[::-1]
            Tdec = float(sp.get("stub_decay_seasons", 8))
            for r, s in zip(roots, mk_stub):
                if not s:
                    continue
                c = int(r)
                for _ in range(nstub):
                    if c < 0:
                        break
                    stub[c] = True
                    c = int(mc[c])
            dec = np.nonzero(stub & self.alive[:n])[0]
            self.alive[dec] = False
            self.dead_since[dec] = season
            # one decay time per stub root, shared along the stub
            draw = rng.uniform(0.5, 1.5, len(roots))
            dmap = np.zeros(n)
            dmap[roots] = draw
            for i in dec:
                j = i
                while dmap[j] == 0 and self.parent[j] >= 0:
                    j = self.parent[j]
                self.decay_at[i] = season + int(round(Tdec * dmap[j]))
            self.remove |= shed & ~stub
        # stubs that have rotted away
        rot = (~self.alive[:n]) & (self.decay_at[:n] >= 0) & (self.decay_at[:n] <= season)
        if rot.any():
            par = self.parent[:n]
            for i in range(1, n):
                if not rot[i] and rot[par[i]]:
                    rot[i] = True
            self.remove |= rot

    def _compact(self):
        n = self.n
        rem = self.remove[:n] if hasattr(self, "remove") and len(self.remove) >= n else np.zeros(n, bool)
        if not rem.any():
            return
        e = float(self.sp["n_pipe"])
        par = self.parent[:n]
        # cut points: removed node whose parent is kept -> conserve its pipe contribution
        cut = np.nonzero(rem & (par >= 0))[0]
        cut = cut[~rem[par[cut]]]
        np.add.at(self.shed_pipe, par[cut], self.radius[cut] ** e)
        keep = ~rem
        newid = -np.ones(n, dtype=np.int64)
        newid[keep] = np.arange(int(keep.sum()))
        m = int(keep.sum())
        for name in self.NODE_FIELDS:
            arr = getattr(self, name)
            arr[:m] = arr[:n][keep]
        p = self.parent[:m]
        self.parent[:m] = np.where(p >= 0, newid[np.maximum(p, 0)], -1)
        self.n = m
        nb = self.nb
        bk = keep[self.b_node[:nb]] & (self.b_state[:nb] != DEAD)
        mb = int(bk.sum())
        for name in self.BUD_FIELDS:
            arr = getattr(self, name)
            arr[:mb] = arr[:nb][bk]
        self.b_node[:mb] = newid[self.b_node[:mb]]
        self.nb = mb
        self.remove = np.zeros(m, dtype=bool)

    def _update_radii(self):
        n = self.n
        e = float(self.sp["n_pipe"])
        rt = float(self.sp["r_tip"])
        par = self.parent[:n]
        alive = self.alive[:n]
        has_child = np.zeros(n, dtype=bool)
        has_child[par[par >= 0]] = True
        old = self.radius[:n].copy()
        levels = self._levels()
        o, bounds = levels
        acc = self.shed_pipe[:n] + np.where(has_child, 0.0, rt ** e)
        out = acc.copy()
        # dead nodes are frozen: they contribute their frozen radius, never recomputed
        out[~alive] = old[~alive] ** e
        for d in range(len(bounds) - 2, 0, -1):
            ids = o[bounds[d]:bounds[d + 1]]
            if len(ids):
                dead = ids[~alive[ids]]
                out[dead] = old[dead] ** e      # frozen dead wood flows its own radius only
                np.add.at(out, par[ids], out[ids])
        r = out ** (1.0 / e)
        r[~alive] = old[~alive]
        # dead parents with live children (rare): keep pipe radius anyway
        self.radius[:n] = np.maximum(r, old)

    # ------------------------------------------------------------------ export
    def finalize_light(self, grid):
        n = self.n
        if grid is not None:
            self.light[:n] = grid.exposure(self.origin + self.pos[:n])

    def to_skeleton(self, extra_meta: dict | None = None) -> Skeleton:
        n = self.n
        sp = self.sp
        r = self.radius[:n].copy()
        z = self.pos[:n, 2]
        f = float(sp.get("flare", 0.0))
        hf = float(sp.get("flare_height", 0.6))
        stem = self.order[:n] == 0
        r[stem] *= 1.0 + f * np.exp(-np.maximum(z[stem], 0) / hf)
        meta = {
            "species": sp["name"],
            "forest_seed": self.forest_seed,
            "tree_id": self.tree_id,
            "params_hash": params_hash(sp),
            "seasons": self.seasons,
            "start_season": self.start_season,
            "n_pipe": float(sp["n_pipe"]),
            "r_tip": float(sp["r_tip"]),
            "flare": f,
            "flare_height": hf,
            "origin_unity": [float(self.origin[0]), float(self.origin[2]), float(self.origin[1])],
        }
        if extra_meta:
            meta.update(extra_meta)
        sk = Skeleton(self.pos[:n].copy(), self.parent[:n].astype(np.int32), r,
                      np.minimum(self.order[:n], 255).astype(np.uint8), self.alive[:n].copy(),
                      self.is_main[:n].copy(), self.birth[:n].astype(np.int16),
                      self.dead_since[:n].astype(np.int16), self.light[:n].astype(np.float32), meta)
        sk.shed_pipe = self.shed_pipe[:n].copy()
        sk.pipe_radius = self.radius[:n].copy()
        return sk


def make_grid(lo, hi, light: dict | None = None) -> ShadowGrid:
    light = dict(light or {})
    return ShadowGrid(lo, hi, **light)


def grow_tree(species: dict, forest_seed: int = 0, tree_id: int = 0, seasons: int | None = None,
              light: dict | None = None) -> Skeleton:
    """Grow a single open-grown tree in its own light field."""
    seasons = int(seasons if seasons is not None else species["seasons"])
    half = float(species.get("crown_radius_max", 8.0)) + 4.0
    H = float(species["height_max"]) * 1.25 + 4.0
    grid = make_grid((-half, -half, -0.5), (half, half, H), light)
    tg = TreeGrower(species, tree_id, forest_seed, seasons=seasons)
    for s in range(seasons):
        grid.compute(tg.active_bud_world_positions())
        tg.step(s, grid)
    grid.compute(tg.active_bud_world_positions())
    tg.finalize_light(grid)
    return tg.to_skeleton()
