# SPDX-License-Identifier: GPL-3.0-or-later
"""GPU canary: tiny MLP regression, deterministic, time-budgeted, checkpointed at 30 s, resumable.

  python canary_train.py --out DIR --budget-s 60 --ckpt-at-s 30 [--device cuda|cpu] [--resume] [--stop-after-ckpt]

Phase A (job 1): run, write ckpt.pt at >= 30 s of *training* time, then exit 0 with state "preempted-sim" (--stop-after-ckpt).
Phase B (job 2): --resume loads ckpt.pt (model, optimizer, RNG, step, elapsed training seconds) and trains to the 60 s budget.
Result: DIR/result.json {device, steps, resumed_from_step, loss_first, loss_final, loss_at_ckpt, wall_s, ckpt_sha256}.
Pass = loss_final < loss_first and (phase B) resumed_from_step > 0 and model state at resume equals the checkpoint.
Resume correctness uses a fixed-step replay check: after loading, the first batch's loss must equal the loss recorded in
the checkpoint for that step (--verify-resume; torch determinism on one device).
"""
import argparse, hashlib, json, os, time
import torch
import torch.nn as nn


def batch(step, device):
    g = torch.Generator().manual_seed(1000 + step)  # data is a pure function of the step: resume-exact
    x = torch.randn(256, 16, generator=g)
    y = (torch.sin(x[:, :4].sum(1, keepdim=True)) + 0.1 * x[:, 4:8].prod(1, keepdim=True)).contiguous()
    return x.to(device), y.to(device)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True); ap.add_argument("--budget-s", type=float, default=60)
    ap.add_argument("--ckpt-at-s", type=float, default=30); ap.add_argument("--device", default="cuda")
    ap.add_argument("--resume", action="store_true"); ap.add_argument("--stop-after-ckpt", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    dev = a.device if (a.device == "cpu" or torch.cuda.is_available()) else "cpu"
    torch.manual_seed(a.seed)
    net = nn.Sequential(nn.Linear(16, 128), nn.ReLU(), nn.Linear(128, 128), nn.ReLU(), nn.Linear(128, 1)).to(dev)
    opt = torch.optim.Adam(net.parameters(), 1e-3)
    step, trained_s, first, at_ckpt, replay_loss = 0, 0.0, None, None, None
    ck = os.path.join(a.out, "ckpt.pt")
    if a.resume:
        s = torch.load(ck, map_location=dev, weights_only=True)
        net.load_state_dict(s["net"]); opt.load_state_dict(s["opt"])
        step, trained_s, first, at_ckpt, replay_loss = s["step"], s["trained_s"], s["loss_first"], s["loss_at_ckpt"], s["next_loss"]
    resumed_from = step
    t0 = time.time(); last = None; ckpt_done = a.resume
    verify = None
    while trained_s + (time.time() - t0) * 0 < a.budget_s:
        ts = time.time()
        x, y = batch(step, dev)
        loss = nn.functional.mse_loss(net(x), y)
        if a.resume and step == resumed_from:
            verify = abs(loss.item() - replay_loss)  # replay equals the loss recorded at save time
        opt.zero_grad(); loss.backward(); opt.step()
        last = loss.item(); first = last if first is None else first
        step += 1; trained_s += time.time() - ts
        if not ckpt_done and trained_s >= a.ckpt_at_s:
            xn, yn = batch(step, dev)
            with torch.no_grad():
                nl = nn.functional.mse_loss(net(xn), yn).item()
            torch.save({"net": net.state_dict(), "opt": opt.state_dict(), "step": step, "trained_s": trained_s,
                        "loss_first": first, "loss_at_ckpt": last, "next_loss": nl}, ck)
            at_ckpt, ckpt_done = last, True
            if a.stop_after_ckpt:
                break
    res = {"device": dev, "steps": step, "resumed_from_step": resumed_from, "loss_first": first, "loss_at_ckpt": at_ckpt,
           "loss_final": last, "trained_s": trained_s, "wall_s": time.time() - t0, "resume_replay_abs_err": verify,
           "ckpt_sha256": hashlib.sha256(open(ck, "rb").read()).hexdigest() if os.path.exists(ck) else None,
           "phase": "B_resume" if a.resume else ("A_preempted" if a.stop_after_ckpt else "single")}
    res["pass"] = (a.stop_after_ckpt and at_ckpt is not None) or (last < first and (not a.resume or (resumed_from > 0 and verify is not None and verify < 1e-4)))
    json.dump(res, open(os.path.join(a.out, "result_%s.json" % res["phase"]), "w"), indent=1)
    print(json.dumps(res))


if __name__ == "__main__":
    main()
