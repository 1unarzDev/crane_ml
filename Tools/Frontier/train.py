# SPDX-License-Identifier: GPL-3.0-or-later
"""Minimal deterministic training loop (NumPy logistic regression; optional torch MLP). Built to run well on CPU.

    python train.py --data X.npz --model logistic --epochs 50 --seed 3 --checkpoint-dir ck --resume
    python train.py --synthetic 2000x8 --model mlp --device cuda          # torch only; falls back with an error if absent

* `--device cpu|cuda`: the NumPy logistic model ignores it (always CPU). `mlp` needs torch; `cuda` needs a CUDA torch.
* Determinism: one `seed` fixes initialisation and the per-epoch shuffles (permutation of `default_rng([seed, epoch])`),
  so a resumed run is bit-identical to an uninterrupted one on CPU (the epoch RNG does not depend on prior epochs).
* Checkpoint/resume: after every `--ckpt-every` epochs, `<dir>/ckpt.npz` (logistic) or `ckpt.pt` (mlp) is written
  atomically (tmp + rename) with the epoch, parameters and optimiser state; `--resume` continues from it. Safe to be
  preempted (jobs.py GPU jobs always pass --resume).
* Data: `.npz` with `X` [n, d] float and `y` [n] in {0, 1}; or `--synthetic NxD` (a seeded linearly-separable-ish task).
"""
import argparse
import json
import os
import sys
import time

import numpy as np

try:
    import torch
except Exception:  # optional
    torch = None


def synthetic(n, d, seed):
    rng = np.random.default_rng([seed, 0xD47A])
    X = rng.normal(size=(n, d))
    w = rng.normal(size=d)
    p = 1 / (1 + np.exp(-(X @ w + 0.5 * X[:, 0] * X[:, 1 % d])))
    return X, (rng.random(n) < p).astype(np.float64)


def _sigmoid(z):
    return 1 / (1 + np.exp(-np.clip(z, -35, 35)))


def _atomic_savez(path, **arrs):
    tmp = path + ".tmp%d.npz" % os.getpid()
    np.savez(tmp, **arrs)
    os.replace(tmp, path)


class Logistic:
    """L2-regularised logistic regression, minibatch SGD with momentum."""

    def __init__(self, d, seed, lr=0.1, l2=1e-4, momentum=0.9):
        self.w, self.b = np.zeros(d), 0.0
        self.vw, self.vb = np.zeros(d), 0.0
        self.lr, self.l2, self.mom = lr, l2, momentum

    def step(self, X, y):
        g = _sigmoid(X @ self.w + self.b) - y
        gw, gb = X.T @ g / len(y) + self.l2 * self.w, g.mean()
        self.vw, self.vb = self.mom * self.vw - self.lr * gw, self.mom * self.vb - self.lr * gb
        self.w, self.b = self.w + self.vw, self.b + self.vb

    def loss(self, X, y):
        p = np.clip(_sigmoid(X @ self.w + self.b), 1e-12, 1 - 1e-12)
        return float(-(y * np.log(p) + (1 - y) * np.log(1 - p)).mean())

    def predict(self, X):
        return _sigmoid(X @ self.w + self.b)

    def state(self):
        return {"w": self.w, "b": np.array(self.b), "vw": self.vw, "vb": np.array(self.vb)}

    def load(self, z):
        self.w, self.b, self.vw, self.vb = z["w"], float(z["b"]), z["vw"], float(z["vb"])


def train_logistic(X, y, epochs=50, seed=0, batch=128, lr=0.1, ckpt_dir=None, resume=False, ckpt_every=1, log=None):
    m, start = Logistic(X.shape[1], seed, lr=lr), 0
    path = os.path.join(ckpt_dir, "ckpt.npz") if ckpt_dir else None
    if resume and path and os.path.exists(path):
        z = np.load(path)
        m.load(z)
        start = int(z["epoch"])
    for ep in range(start, epochs):
        perm = np.random.default_rng([seed, ep]).permutation(len(y))
        for i in range(0, len(y), batch):
            idx = perm[i:i + batch]
            m.step(X[idx], y[idx])
        if log:
            log({"epoch": ep + 1, "loss": m.loss(X, y)})
        if path and (ep + 1) % ckpt_every == 0:
            os.makedirs(ckpt_dir, exist_ok=True)
            _atomic_savez(path, epoch=np.array(ep + 1), **m.state())
    return m


def train_mlp(X, y, epochs=50, seed=0, batch=128, lr=0.05, hidden=32, device="cpu", ckpt_dir=None, resume=False,
              ckpt_every=1, log=None):
    if torch is None:
        raise RuntimeError("torch is not importable; use --model logistic")
    if device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("--device cuda requested but CUDA is not available")
    torch.manual_seed(seed)
    torch.use_deterministic_algorithms(True, warn_only=True)
    net = torch.nn.Sequential(torch.nn.Linear(X.shape[1], hidden), torch.nn.Tanh(), torch.nn.Linear(hidden, 1)).to(device)
    opt = torch.optim.SGD(net.parameters(), lr=lr, momentum=0.9)
    Xt, yt = torch.tensor(X, dtype=torch.float32, device=device), torch.tensor(y, dtype=torch.float32, device=device)
    path, start = (os.path.join(ckpt_dir, "ckpt.pt") if ckpt_dir else None), 0
    if resume and path and os.path.exists(path):
        ck = torch.load(path, map_location=device)
        net.load_state_dict(ck["net"])
        opt.load_state_dict(ck["opt"])
        start = ck["epoch"]
    lossf = torch.nn.BCEWithLogitsLoss()
    for ep in range(start, epochs):
        perm = torch.tensor(np.random.default_rng([seed, ep]).permutation(len(y)), device=device)
        for i in range(0, len(y), batch):
            idx = perm[i:i + batch]
            opt.zero_grad()
            lossf(net(Xt[idx]).squeeze(-1), yt[idx]).backward()
            opt.step()
        if log:
            with torch.no_grad():
                log({"epoch": ep + 1, "loss": float(lossf(net(Xt).squeeze(-1), yt))})
        if path and (ep + 1) % ckpt_every == 0:
            os.makedirs(ckpt_dir, exist_ok=True)
            tmp = path + ".tmp%d" % os.getpid()
            torch.save({"net": net.state_dict(), "opt": opt.state_dict(), "epoch": ep + 1}, tmp)
            os.replace(tmp, path)
    return net


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--data")
    ap.add_argument("--synthetic", help="NxD, e.g. 2000x8")
    ap.add_argument("--model", choices=["logistic", "mlp"], default="logistic")
    ap.add_argument("--device", choices=["cpu", "cuda"], default="cpu")
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--batch", type=int, default=128)
    ap.add_argument("--lr", type=float, default=None)
    ap.add_argument("--hidden", type=int, default=32)
    ap.add_argument("--checkpoint-dir")
    ap.add_argument("--ckpt-every", type=int, default=1)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--out", help="write metrics JSON here")
    a = ap.parse_args(argv)
    if a.data:
        z = np.load(a.data)
        X, y = z["X"].astype(np.float64), z["y"].astype(np.float64)
    elif a.synthetic:
        n, d = (int(v) for v in a.synthetic.split("x"))
        X, y = synthetic(n, d, a.seed)
    else:
        ap.error("--data or --synthetic required")
    hist = []
    t0 = time.time()
    if a.model == "logistic":
        m = train_logistic(X, y, a.epochs, a.seed, a.batch, a.lr or 0.1, a.checkpoint_dir, a.resume, a.ckpt_every, hist.append)
        acc = float(((m.predict(X) > 0.5) == (y > 0.5)).mean())
    else:
        net = train_mlp(X, y, a.epochs, a.seed, a.batch, a.lr or 0.05, a.hidden, a.device, a.checkpoint_dir, a.resume,
                        a.ckpt_every, hist.append)
        with torch.no_grad():
            pr = net(torch.tensor(X, dtype=torch.float32, device=a.device)).squeeze(-1).cpu().numpy()
        acc = float(((pr > 0) == (y > 0.5)).mean())
    res = {"model": a.model, "device": a.device if a.model == "mlp" else "cpu", "seed": a.seed, "epochs": a.epochs,
           "train_acc": acc, "final_loss": hist[-1]["loss"] if hist else None, "wall_s": round(time.time() - t0, 3)}
    print(json.dumps(res))
    if a.out:
        json.dump(res, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
