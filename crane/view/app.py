"""``crane view``: open runs, or a live sim, in the native window; or render it offscreen to a PNG.

    crane view runs/boat/base/s3 unity.ctraj            replay, overlaid in one frame and on one clock
    crane view --live --domain boat --tier T1 --seed 3  the sim in-process, with sliders
    crane view RUN --screenshot /tmp/v.png --at 60      offscreen (QT_QPA_PLATFORM=offscreen), never a window
"""

from __future__ import annotations

import json
import os
import sys


def _need_qt():
    try:
        import PySide6  # noqa: F401
        import pyqtgraph  # noqa: F401
    except ImportError as error:
        print(f"crane view: needs the [view] extra ({error.name} is missing): "
              "uv pip install -e '.[view]'  (or pip install -e '.[view]')", file=sys.stderr)
        return False
    return True


def build(args, log=print):
    """(QApplication, Viewer) for parsed ``crane view`` arguments."""
    from crane import tiers
    from crane.view import scene as vscene
    from PySide6 import QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(['crane view'])
    sc = None
    if args.runs:
        transforms = [json.loads(t) if t else None for t in (args.transform or [])]
        offsets = [float(o) if o not in (None, '') else None for o in (args.offset or [])]
        sc = vscene.load(args.runs, transforms=[None] + transforms, offsets=[None] + offsets)
    live = None
    if args.live:
        from crane.view.live import FastsimLive
        tier = tiers.normalise(args.tier)
        if tier == 'T2':
            raise vscene.SceneError('live mode runs T0 or T1 in-process; T2 is the Unity sim (crane sim up --window)')
        live = FastsimLive.for_domain(args.domain, tier, args.seed)
        if live.generated:
            log(f'crane view: wrote the missing scenario for seed {args.seed} ({live.source.path(args.seed)})')
    if sc is None and live is None:
        raise vscene.SceneError('give run paths, or --live --domain D --tier T --seed N')
    from crane.view.window import Viewer
    win = Viewer(sc, live, three_d=args.three_d, promote_root=args.promote_dir, log=log)
    return app, win


def screenshot(args, log=print) -> int:
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    app, win = build(args, log)
    win.resize(1500, 920)
    if win.sim is not None:
        win._advance_live(10 ** 9 if args.at is None else max(0, round(args.at / win.sim.dt)))
    if args.at is not None:
        win.set_time(args.at)
    if args.three_d:
        log('crane view: the 3-D view needs an OpenGL display, which offscreen rendering has not; '
            'the screenshot shows the 2-D map')
    win.show()
    for _ in range(5):
        app.processEvents()
    win.set_time(win.t)
    app.processEvents()
    ok = win.grab().save(args.screenshot)
    win.close()
    if not ok:
        print(f'crane view: could not write {args.screenshot}', file=sys.stderr)
        return 1
    log(f'crane view: {args.screenshot} at t = {win.t:.1f} s')
    return 0


def main(args) -> int:
    if args.screenshot:
        os.environ['QT_QPA_PLATFORM'] = 'offscreen'    # before Qt loads: no window, ever
    if not _need_qt():
        return 2
    from crane.view.scene import SceneError
    from crane.record import RecordError
    from crane.frames import FrameError
    try:
        if args.screenshot:
            return screenshot(args)
        if os.environ.get('QT_QPA_PLATFORM') != 'offscreen' and not (os.environ.get('DISPLAY') or
                                                                      os.environ.get('WAYLAND_DISPLAY')):
            print('crane view: no display; use --screenshot OUT.png (offscreen)', file=sys.stderr)
            return 2
        app, win = build(args)
    except (SceneError, RecordError, FrameError, LookupError, ValueError, FileNotFoundError) as error:
        print(f'crane view: {error}', file=sys.stderr)
        return 2
    if args.three_d:
        win.show_3d()
    win.show()
    return app.exec()
