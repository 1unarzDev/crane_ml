"""Standard noise-free benchmarks on the unit cube (minimisation), used for baseline acceptance."""
import numpy as np

from .optimizers import make_optimizer

_A = np.array([[10, 3, 17, 3.5, 1.7, 8], [0.05, 10, 17, 0.1, 8, 14], [3, 3.5, 1.7, 10, 17, 8],
               [17, 8, 0.05, 10, 0.1, 14]], float)
_P = 1e-4 * np.array([[1312, 1696, 5569, 124, 8283, 5886], [2329, 4135, 8307, 3736, 1004, 9991],
                      [2348, 1451, 3522, 2883, 3047, 6650], [4047, 8828, 8732, 5743, 1091, 381]], float)
_ALPHA = np.array([1.0, 1.2, 3.0, 3.2])


def branin(u):
    x1, x2 = -5 + 15 * u[0], 15 * u[1]
    return float((x2 - 5.1 / (4 * np.pi ** 2) * x1 ** 2 + 5 / np.pi * x1 - 6) ** 2
                 + 10 * (1 - 1 / (8 * np.pi)) * np.cos(x1) + 10)


def hartmann6(u):
    return float(-(_ALPHA * np.exp(-(_A * (np.asarray(u) - _P) ** 2).sum(1))).sum())


def ellipsoid(u):
    n = len(u)
    z = (np.asarray(u) - (0.37 + 0.05 * (np.arange(n) % 3))) * 10.0
    return float((10.0 ** (6 * np.arange(n) / (n - 1)) * z ** 2).sum())


def rosenbrock(u):
    x = -2 + 4 * np.asarray(u)
    return float((100 * (x[1:] - x[:-1] ** 2) ** 2 + (1 - x[:-1]) ** 2).sum())


FMIN = {"branin": 0.397887, "hartmann6": -3.32237, "ellipsoid": 0.0, "rosenbrock": 0.0}
FUNCS = {"branin": branin, "hartmann6": hartmann6, "ellipsoid": ellipsoid, "rosenbrock": rosenbrock}


def run(name, optimizer, dim, n_evals, seed):
    """Return simple regret (best evaluated f - f*) after n_evals."""
    f = FUNCS[name]
    o = make_optimizer(optimizer, dim=dim, seed=seed, x0=np.full(dim, 0.5), ctx_dim=0,
                       fidelities={"sim_high": 1.0}, default_fidelity="sim_high")
    best = float("inf")
    for _ in range(n_evals):
        x, fid = o.ask(None)
        v = f(x)
        best = min(best, v)
        o.tell(x, None, fid, -v)
    return best - FMIN[name]


# ---- multi-fidelity: Forrester (Forrester, Sobester & Keane 2007), published low-fidelity variant
def forrester_high(x):
    return float((6 * x - 2) ** 2 * np.sin(12 * x - 4))


def forrester_low(x):
    return 0.5 * forrester_high(x) + 10 * (x - 0.5) - 5


FORRESTER_FMIN = -6.02074
FORRESTER_COST = {"sim_low": 0.1, "sim_high": 1.0}     # cost ratio 1:10


def run_forrester(optimizer, budget, seed, **kw):
    """Equal total cost; returns high-fidelity regret of recommend() (noise-free, so no winner's curse)."""
    o = make_optimizer(optimizer, dim=1, seed=seed, x0=np.array([0.5]), ctx_dim=0,
                       fidelities=dict(FORRESTER_COST), default_fidelity="sim_high", **kw)
    spent = 0.0
    while True:
        x, fid = o.ask(None)
        c = FORRESTER_COST[fid]
        if spent + c > budget + 1e-9:
            break
        spent += c
        f = forrester_high if fid == "sim_high" else forrester_low
        o.tell(x, None, fid, -f(float(x[0])))
    return forrester_high(float(o.recommend(None)[0])) - FORRESTER_FMIN


# ---- contextual: the optimum moves linearly with the context
def ctx_optimum(c):
    return np.array([0.2 + 0.6 * c, 0.8 - 0.6 * c])


def ctx_reward(x, c):
    return -float(((np.asarray(x) - ctx_optimum(c)) ** 2).sum())


TRAIN_CTX = [0.1, 0.3, 0.5, 0.7, 0.9]
HELDOUT_CTX = [0.2, 0.4, 0.6, 0.8]


def run_contextual(optimizer, n_trials, seed):
    o = make_optimizer(optimizer, dim=2, seed=seed, x0=np.array([0.5, 0.5]), ctx_dim=1,
                       fidelities={"sim_high": 1.0}, default_fidelity="sim_high")
    for i in range(n_trials):
        c = TRAIN_CTX[i % len(TRAIN_CTX)]
        x, fid = o.ask(np.array([c]))
        o.tell(x, np.array([c]), fid, ctx_reward(x, c))
    return float(np.mean([-ctx_reward(o.recommend(np.array([c])), c) for c in HELDOUT_CTX]))
