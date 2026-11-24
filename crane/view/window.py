"""The native sandbox window (PySide6 + pyqtgraph): a top-down map, a timeline of event lanes, the play controls
and, in live mode, the parameter sliders. Only ``crane view`` imports this; the core never does.

Map: world layers (course elements, gates, footprints, boundary, constraint volumes), each run's trail up to the
playhead (later samples faint, constraint violations red), the vehicle outline at its heading. Timeline: one row
per event lane (mission, goal, judge, log, ...), spans as bars, the playhead draggable. A 3-D view
(GLViewWidget) shows the trails with z for 6-DOF records.
"""

from __future__ import annotations

import math
import time

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

from crane.view import layers as vlayers, scene as vscene
from crane.view.live import Snapshots

BG, FG, DIM = '#0b1220', '#c3c2b7', '#3a4e6b'
RUN_COLORS = ['#3FC8E0', '#C792EA', '#F2A65A', '#5FD38A', '#FF6B8B', '#F2D45C']
STYLE = (f'QWidget {{ background:#0e1726; color:{FG}; }} QPlainTextEdit {{ background:{BG}; font-family:monospace; '
         'font-size:11px; } QPushButton { background:#17263d; border:1px solid #2a3d5c; padding:4px 10px; '
         'border-radius:3px; } QPushButton:disabled { color:#4a5568; } QPushButton:checked { background:#2a6a7d; } '
         'QGroupBox { border:1px solid #22324a; margin-top:14px; padding-top:6px; } '
         'QGroupBox::title { subcontrol-origin: margin; left:6px; }')
LANE_COLORS = {'mission': '#7DD3DC', 'goal': '#F2D45C', 'judge': '#ff5a52', 'log': '#9ec5f4', 'events': '#c3c2b7'}


def element_color(label: str) -> str:
    l = (label or '').lower()
    for k, c in (('red', '#ff5a52'), ('green', '#36D399'), ('yellow', '#F2D45C'), ('black', '#8a93a6'),
                 ('blue', '#4F7BFF'), ('white', '#f0efec'), ('orange', '#f28f63')):
        if l.startswith(k):
            return c
    return '#9aa3b2'


def _pen(color, width=1.0, style=None, alpha=255):
    c = QtGui.QColor(color)
    c.setAlpha(alpha)
    p = pg.mkPen(c, width=width)
    if style:
        p.setStyle(style)
    p.setCosmetic(True)
    return p


def _brush(color, alpha):
    c = QtGui.QColor(color)
    c.setAlpha(alpha)
    return QtGui.QBrush(c)


class Display:
    """World (scene frame) -> map axes: ENU draws x east right, y north up; NED is swapped to the same picture."""

    def __init__(self, frame):
        self.ned = frame.get('world') == 'ned'

    def xy(self, x, y):
        return (y, x) if self.ned else (x, y)

    def yaw(self, yaw):
        return math.pi / 2 - yaw if self.ned else yaw


class Viewer(QtWidgets.QMainWindow):
    def __init__(self, scene=None, live=None, *, three_d=False, promote_root='runs/promoted', log=print):
        super().__init__()
        self.scene, self.sim, self.log = scene, live, log
        self.promote_root = promote_root
        self.recorded = list(scene.runs) if scene else []
        frame = scene.frame if scene else live.frame
        if scene and live and vscene.frames.frame_of(scene.frame)['origin'] != vscene.frames.frame_of(live.frame)['origin']:
            raise vscene.SceneError(f"the live sim's origin {live.frame['origin']} differs from the runs' "
                                    f"{scene.frame['origin']}: overlay refused")
        self.disp = Display(frame)
        self.outline = (scene.outline if scene else live.spec['outline'])
        self.snaps = Snapshots(1.0) if live else None
        self.t, self.playing, self.speed = 0.0, False, 4.0
        self._wall = time.perf_counter()
        self._live_events = -1
        self.setWindowTitle('crane view')
        self.resize(1500, 920)
        pg.setConfigOptions(antialias=True, background=BG, foreground=FG)
        self.setStyleSheet(STYLE)
        self._build(three_d)
        if live:
            self.snaps.keep(live)
        self.runs = self._runs()
        self._draw_world()
        self._build_run_items()
        self._fit()
        self.set_time(self.t_range[1] if not live else 0.0)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(33)

    # ---- layout ------------------------------------------------------------------------------------------------

    def _build(self, three_d):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        outer = QtWidgets.QVBoxLayout(central)
        outer.setContentsMargins(6, 6, 6, 6)
        self.banner = QtWidgets.QLabel()
        self.banner.setWordWrap(True)
        self.banner.setStyleSheet(f'color:{FG}; background:#132036; padding:5px 8px; border-radius:4px;')
        outer.addWidget(self.banner)
        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        outer.addWidget(split, 1)
        left = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        split.addWidget(left)
        self.stack = QtWidgets.QStackedWidget()
        self.map = pg.PlotWidget()
        self.map.setAspectLocked(True)
        self.map.showGrid(x=True, y=True, alpha=0.15)
        n, e = ('north (x) m', 'east (y) m') if self.disp.ned else ('y north m', 'x east m')
        self.map.setLabel('left', n)
        self.map.setLabel('bottom', e)
        self.stack.addWidget(self.map)
        self.gl = None
        left.addWidget(self.stack)
        self.tl = pg.PlotWidget()
        self.tl.setMouseEnabled(x=True, y=False)
        self.tl.setLabel('bottom', 'time s')
        self.tl.hideButtons()
        left.addWidget(self.tl)
        left.setSizes([640, 220])
        side = QtWidgets.QWidget()
        side.setMinimumWidth(330)
        side.setMaximumWidth(400)
        split.addWidget(side)
        self.side = QtWidgets.QVBoxLayout(side)
        self.info = QtWidgets.QLabel()
        self.info.setWordWrap(True)
        self.info.setTextFormat(QtCore.Qt.RichText)
        self.info.setAlignment(QtCore.Qt.AlignTop)
        self.side.addWidget(self.info)
        self.params_box = QtWidgets.QGroupBox('Parameters (live)')
        self.params_form = QtWidgets.QFormLayout(self.params_box)
        self.side.addWidget(self.params_box)
        self.params_box.setVisible(self.sim is not None)
        self.sliders = {}
        if self.sim is not None:
            for p in self.sim.declared():
                self._slider(p)
        self.events_box = QtWidgets.QPlainTextEdit()
        self.events_box.setReadOnly(True)
        self.events_box.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.events_box.setMaximumBlockCount(400)
        self.side.addWidget(QtWidgets.QLabel('Events up to the playhead'))
        self.side.addWidget(self.events_box, 1)
        bar = QtWidgets.QHBoxLayout()
        outer.addLayout(bar)
        self.play_btn = self._button(bar, 'Play', self.toggle)
        self._button(bar, 'Step', self.step_once)
        bar.addWidget(QtWidgets.QLabel('speed'))
        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(-2, 6)           # 2^k x
        self.speed_slider.setValue(2)
        self.speed_slider.setFixedWidth(120)
        self.speed_slider.valueChanged.connect(self._speed)
        bar.addWidget(self.speed_slider)
        self.speed_label = QtWidgets.QLabel('4x')
        bar.addWidget(self.speed_label)
        self.scrub = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.scrub.setRange(0, 1000)
        self.scrub.sliderMoved.connect(self._scrubbed)
        bar.addWidget(self.scrub, 1)
        self.clock = QtWidgets.QLabel()
        self.clock.setMinimumWidth(130)
        bar.addWidget(self.clock)
        self.rerun_t = self._button(bar, 'Rerun from t', self.rerun_from_t)
        self.rerun_0 = self._button(bar, 'Rerun from 0', self.rerun_from_0)
        self.promote_btn = self._button(bar, 'Promote', self.promote)
        self.three_btn = self._button(bar, '3-D', self.toggle_3d)
        self.three_btn.setCheckable(True)
        self.rerun_t.setEnabled(self.sim is not None)
        self.promote_btn.setEnabled(self.sim is not None)
        rerun0 = self.sim is not None or any(self._config_of(r) for r in self._recorded_runs())
        self.rerun_0.setEnabled(rerun0)
        if self.sim is None:
            self.rerun_t.setToolTip('Recorded runs hold poses only (no sim state): rerun is from 0 only')
        self.status = QtWidgets.QLabel()
        self.status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.statusBar().addWidget(self.status, 1)
        self.playhead_tl = pg.InfiniteLine(0.0, angle=90, movable=True, pen=_pen('#7DD3DC', 1.5))
        self.playhead_tl.sigDragged.connect(lambda line: self.set_time(float(line.value()), from_line=True))
        self.three_d_wanted = three_d

    def _button(self, bar, text, fn):
        b = QtWidgets.QPushButton(text)
        b.clicked.connect(fn)
        bar.addWidget(b)
        return b

    def _slider(self, p):
        row = QtWidgets.QWidget()
        lay = QtWidgets.QHBoxLayout(row)
        lay.setContentsMargins(0, 0, 0, 0)
        s = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        n = max(1, round((p['max'] - p['min']) / (p.get('step') or (p['max'] - p['min']) / 100)))
        s.setRange(0, n)
        val = self.sim.params()[p['name']]
        s.setValue(round((val - p['min']) / (p['max'] - p['min']) * n))
        label = QtWidgets.QLabel(self._fmt(p, val))
        label.setMinimumWidth(78)
        lay.addWidget(s, 1)
        lay.addWidget(label)

        def changed(k, p=p, n=n, label=label):
            v = round(p['min'] + (p['max'] - p['min']) * k / n, 6)
            label.setText(self._fmt(p, v))
            self.sim.set_param(p['name'], v)
            self._status(f"{p['name']} = {v:g} from t = {self.sim.t:.1f} s (Rerun from t / 0 to apply earlier)")
        s.valueChanged.connect(changed)
        self.sliders[p['name']] = s
        self.params_form.addRow(p.get('label', p['name']), row)

    @staticmethod
    def _fmt(p, v):
        return f"{v:g} {p.get('unit', '')}".strip()

    # ---- runs --------------------------------------------------------------------------------------------------

    def _recorded_runs(self):
        return self.recorded

    def _runs(self):
        runs = list(self.recorded)
        if self.sim is not None:
            runs.append(vscene.live_run(self.sim, f'live {self.sim.tier} {self.sim.domain} s{self.sim.seed}'))
        return runs

    @property
    def t_range(self):
        lo, hi = 0.0, 1.0
        ts = [r.t for r in self.runs if len(r.t)]
        evs = [e['t'] for r in self.runs for e in r.events]
        if ts or evs:
            lo = min([float(t[0]) for t in ts] + evs + [0.0])
            hi = max([float(t[-1]) for t in ts] + evs + [lo + 1.0])
        return lo, hi

    def _world_layers(self):
        base = next((r.world for r in self.runs if r.world), [])
        return base + [c for r in self.runs for c in r.layers]

    def _poly_item(self, pts, pen, brush=None):
        d = self.disp
        poly = QtGui.QPolygonF([QtCore.QPointF(*d.xy(float(x), float(y))) for x, y in pts])
        item = QtWidgets.QGraphicsPolygonItem(poly)
        item.setPen(pen)
        item.setBrush(brush or QtGui.QBrush(QtCore.Qt.NoBrush))
        self.map.addItem(item)
        return item

    def _draw_world(self):
        d = self.disp
        for layer in self._world_layers():
            t = layer.get('type')
            if t == 'polygons':
                boundary = layer.get('name') == 'boundary'
                for poly in layer.get('polys') or []:
                    self._poly_item(poly, _pen('#4a5568' if boundary else '#1f6f8b', 1, QtCore.Qt.DashLine),
                                    None if boundary else _brush('#3FC8E0', 12))
            elif t == 'segments':
                for (x0, y0), (x1, y1) in layer.get('items') or []:
                    a, b = d.xy(x0, y0), d.xy(x1, y1)
                    self.map.plot([a[0], b[0]], [a[1], b[1]], pen=_pen('#f0efec', 1, QtCore.Qt.DotLine, 90))
            elif t == 'disks':
                items = layer.get('items') or []
                if items:
                    xy = [d.xy(e['x'], e['y']) for e in items]
                    sp = pg.ScatterPlotItem(x=[p[0] for p in xy], y=[p[1] for p in xy],
                                            size=[max(0.2, 2 * e['r']) for e in items], pxMode=False,
                                            brush=[_brush(element_color(e['label']), 220 if e['label'] else 35)
                                                   for e in items],
                                            pen=_pen('#ffffff', 0.5, alpha=60), hoverable=True,
                                            data=[e.get('id') or e['label'] for e in items],
                                            tip=lambda x, y, data: str(data))
                    self.map.addItem(sp)
            elif t == 'constraint':
                keep_out = layer['mode'] == 'keep_out'
                col = '#ff5a52' if keep_out else '#36D399'
                self._poly_item(vlayers.outline(layer), _pen(col, 1.6, None if keep_out else QtCore.Qt.DashLine),
                                _brush(col, 55 if keep_out else 18))
                if layer.get('label') or layer.get('id'):
                    x, y = d.xy(*vlayers.outline(layer)[0])
                    txt = pg.TextItem(str(layer.get('label') or layer.get('id')), color=col, anchor=(0, 1))
                    txt.setPos(x, y)
                    self.map.addItem(txt)

    def _build_run_items(self):
        self.items = []
        for i, r in enumerate(self.runs):
            col = RUN_COLORS[i % len(RUN_COLORS)]
            future = self.map.plot([], [], pen=_pen(col, 1, alpha=50))
            trail = self.map.plot([], [], pen=_pen(col, 2.2), name=r.name)
            bad = pg.ScatterPlotItem(size=5, brush=_brush('#ff5a52', 230), pen=None)
            self.map.addItem(bad)
            hull = self._poly_item(self.outline, _pen(col, 1.6), _brush('#0F1217', 230))
            hull.setZValue(10)
            heading = self.map.plot([], [], pen=_pen(col, 1.2))
            dot = pg.ScatterPlotItem(size=7, brush=_brush(col, 255), pen=_pen('#ffffff', 1))
            dot.setZValue(11)
            self.map.addItem(dot)
            self.items.append({'future': future, 'trail': trail, 'bad': bad, 'hull': hull, 'heading': heading, 'dot': dot,
                               'viol': r.violations(), 'color': col})
        self._build_timeline()

    def _refresh_live(self):
        """Re-read the live run (its trail grows; its events may have changed after a rerun)."""
        if self.sim is None:
            return
        live = vscene.live_run(self.sim, self.runs[-1].name)
        self.runs[-1] = live
        self.items[-1]['viol'] = live.violations()
        it = self.items[-1]
        it['future'].setData(*self._xy(live.x, live.y))
        if len(live.events) != self._live_events:
            self._live_events = len(live.events)
            self._build_timeline()

    def _xy(self, x, y):
        return (y, x) if self.disp.ned else (x, y)

    def _build_timeline(self):
        self.tl.clear()
        lanes = vscene.lanes_of(self.runs) or ['events']
        self.lanes = lanes
        ax = self.tl.getAxis('left')
        ax.setTicks([[(-k, name) for k, name in enumerate(lanes)]])
        self.tl.setYRange(-len(lanes) + 0.4, 0.6, padding=0)
        lo, hi = self.t_range
        self.tl.setXRange(lo, hi, padding=0.01)
        n = len(self.runs)
        for i, r in enumerate(self.runs):
            col = RUN_COLORS[i % len(RUN_COLORS)]
            dy = (i - (n - 1) / 2) * 0.18
            pts, spans = [], []
            for e in r.events:
                y = -lanes.index(e['lane']) + dy
                if 't_end' in e:
                    spans.append((e['t'], e['t_end'], y))
                pts.append((e['t'], y, e))
            for t0, t1, y in spans:
                bar = QtWidgets.QGraphicsRectItem(t0, y - 0.07, max(1e-3, t1 - t0), 0.14)
                bar.setBrush(_brush(col, 90))
                bar.setPen(QtGui.QPen(QtCore.Qt.NoPen))
                self.tl.addItem(bar)
            if pts:
                sp = pg.ScatterPlotItem(x=[p[0] for p in pts], y=[p[1] for p in pts], size=8,
                                        symbol=['d' if p[2]['lane'] != 'log' else 'o' for p in pts],
                                        brush=[_brush(LANE_COLORS.get(p[2]['lane'], col), 230) for p in pts],
                                        pen=_pen(col, 1), hoverable=True, data=[self._event_text(p[2]) for p in pts],
                                        tip=lambda x, y, data: f'{x:.1f} s  {data}')
                self.tl.addItem(sp)
            last = -1e9
            for t, y, e in pts:        # log lines: their text beside the marker, where there is room
                if e['lane'] == 'log' and t - last > (hi - lo) / 12:
                    txt = pg.TextItem(str(e.get('text', ''))[:28], color=LANE_COLORS['log'], anchor=(0, 0.5))
                    txt.setPos(t, y)
                    self.tl.addItem(txt)
                    last = t
        self.tl.addItem(self.playhead_tl)

    @staticmethod
    def _event_text(e):
        skip = {'t', 'kind', 'lane', 't_end'}
        rest = ' '.join(f'{k}={v}' for k, v in e.items() if k not in skip and k != 'text')
        return f"{e['kind']}: {e.get('text', '')} {rest}".strip()

    def _fit(self):
        xs, ys = [], []
        for r in self.runs:
            if len(r.x):
                a, b = self._xy(r.x, r.y)
                xs += [float(np.min(a)), float(np.max(a))]
                ys += [float(np.min(b)), float(np.max(b))]
        for layer in self._world_layers():
            if layer.get('type') == 'disks':
                for e in layer['items']:
                    a, b = self.disp.xy(e['x'], e['y'])
                    xs.append(a)
                    ys.append(b)
        if xs:
            self.map.setRange(xRange=(min(xs) - 3, max(xs) + 3), yRange=(min(ys) - 3, max(ys) + 3), padding=0.02)

    # ---- time --------------------------------------------------------------------------------------------------

    def set_time(self, t, from_line=False):
        lo, hi = self.t_range
        if self.sim is not None:
            hi = max(hi, self.sim.t)
        self.t = float(min(max(t, lo), hi))
        for r, it in zip(self.runs, self.items):
            k = r.index_at(self.t)
            x, y = self._xy(r.x, r.y)
            it['future'].setData(x, y)
            it['trail'].setData(x[:k + 1], y[:k + 1])
            viol = it['viol'][:k + 1]
            it['bad'].setData(x[:k + 1][viol], y[:k + 1][viol])
            if k >= 0:
                px, py = self.disp.xy(float(r.x[k]), float(r.y[k]))
                yaw = self.disp.yaw(float(r.yaw[k]))
                tr = QtGui.QTransform()
                tr.translate(px, py)
                tr.rotateRadians(yaw)
                if self.disp.ned:
                    tr.scale(1, -1)
                it['hull'].setTransform(tr)
                it['hull'].setVisible(True)
                L = 1.5 * max(math.hypot(*p) for p in self.outline)
                it['heading'].setData([px, px + L * math.cos(yaw)], [py, py + L * math.sin(yaw)])
                it['dot'].setData([px], [py])
            else:
                it['hull'].setVisible(False)
                it['heading'].setData([], [])
                it['dot'].setData([], [])
        if self.sim is not None and self.t > self.tl.viewRange()[0][1]:
            self.tl.setXRange(lo, self.t + 0.25 * (self.t - lo + 10), padding=0)
        if not from_line:
            self.playhead_tl.setValue(self.t)
        self.scrub.blockSignals(True)
        self.scrub.setValue(int(round((self.t - lo) / ((hi - lo) or 1.0) * 1000)))
        self.scrub.blockSignals(False)
        self.clock.setText(f'{self.t:7.1f} / {hi:.1f} s')
        self._panel()
        if self.gl is not None and self.stack.currentWidget() is self.gl:
            self._draw_3d()

    def _scrubbed(self, v):
        lo, hi = self.t_range
        if self.sim is not None:
            hi = max(hi, self.sim.t)
        self.set_time(lo + (hi - lo) * v / 1000.0)

    def _speed(self, k):
        self.speed = 2.0 ** k
        self.speed_label.setText(f'{self.speed:g}x')

    def toggle(self):
        self.playing = not self.playing
        self.play_btn.setText('Pause' if self.playing else 'Play')
        self._wall = time.perf_counter()

    def step_once(self):
        if self.sim is not None and self.t >= self.sim.t - 1e-9:
            self._advance_live(max(1, round(0.1 / self.sim.dt)))
        else:
            r = next((r for r in self.runs if len(r.t) > 1), None)
            self.set_time(self.t + (float(r.t[1] - r.t[0]) if r else 0.1))

    def _advance_live(self, steps):
        sim = self.sim
        left = steps
        while left > 0 and not sim.done:
            chunk = min(left, max(1, round(1.0 / sim.dt)))
            left -= sim.step(chunk)
            self.snaps.keep(sim)
        self._refresh_live()
        self.set_time(sim.t)

    def _tick(self):
        now = time.perf_counter()
        dt, self._wall = min(0.25, now - self._wall), now
        if not self.playing:
            return
        target = self.t + dt * self.speed
        if self.sim is not None and target > self.sim.t:
            if self.sim.done:
                self.set_time(self.sim.t)
                self.toggle()
                return
            self._advance_live(max(1, round((target - self.sim.t) / self.sim.dt)))
            return
        lo, hi = self.t_range
        if target >= hi and self.sim is None:
            self.set_time(hi)
            self.toggle()
            return
        self.set_time(target)

    # ---- live actions ------------------------------------------------------------------------------------------

    def rerun_from_t(self):
        if self.sim is None:
            self._status('Recorded runs hold poses only: rerun is from 0 only.')
            return
        hit = self.snaps.at_or_before(self.t)
        if hit is None:
            return self.rerun_from_0()
        ts, snap = hit
        self.sim.restore(snap)
        self.snaps.drop_after(ts)
        self._live_events = -1
        self._refresh_live()
        self.set_time(self.sim.t)
        self._status(f'rerun from t = {ts:.1f} s (the snapshot at or before the playhead) with the current sliders')

    def rerun_from_0(self):
        if self.sim is None:
            run = next((r for r in self._recorded_runs() if self._config_of(r)), None)
            if run is None:
                self._status('No recorded run here names a fastsim config: nothing to re-simulate.')
                return
            self._go_live(run)
            return
        self.sim.reset()
        self.snaps = Snapshots(1.0)
        self.snaps.keep(self.sim)
        self._live_events = -1
        self._refresh_live()
        self.set_time(0.0)
        self._status('rerun from 0 with the current sliders')

    @staticmethod
    def _config_of(run):
        """(config path, seed, tier, domain) a recorded fastsim run names, else None."""
        rec = run.record
        if rec is None:
            return None
        prov = rec.meta.get('provenance') or {}
        cfg = prov.get('config_file')
        if not cfg or rec.meta.get('seed') is None:
            return None
        from pathlib import Path
        if not Path(cfg).is_file():
            return None
        tier = rec.meta.get('tier')
        try:
            import json
            native = json.loads((rec.path / 'run.json').read_text())
            tier = ((native.get('config') or {}).get('adapter') or {}).get('options', {}).get('tier') or tier
        except (OSError, ValueError, AttributeError):
            pass
        return cfg, int(rec.meta['seed']), tier, rec.meta.get('domain') or ''

    def _go_live(self, run):
        from crane.view import spec as vspec
        from crane.view.live import FastsimLive
        cfg, seed, tier, domain = self._config_of(run)
        self.sim = FastsimLive(cfg, seed, spec=vspec.view_spec(domain or None), tier=tier, domain=domain)
        self.snaps = Snapshots(1.0)
        self.snaps.keep(self.sim)
        for p in self.sim.declared():
            self._slider(p)
        self.params_box.setVisible(True)
        self.rerun_t.setEnabled(True)
        self.promote_btn.setEnabled(True)
        self.runs = self._runs()
        self.map.clear()
        self._draw_world()
        self._build_run_items()
        self.set_time(0.0)
        self._status(f'live from 0: {cfg} seed {seed} ({tier}), beside the recorded runs')

    def promote(self):
        if self.sim is None:
            return
        from crane.view.live import promote
        path, command = promote(self.sim, self.promote_root)
        self.log(f'crane view: wrote {path}\n{command}')
        self._status(f'promoted: {path} · {command}')
        QtWidgets.QApplication.clipboard().setText(command)

    def _status(self, text):
        self.status.setText(text)

    # ---- panels ------------------------------------------------------------------------------------------------

    def _panel(self):
        rows = []
        for r, it in zip(self.runs, self.items):
            k = r.index_at(self.t)
            pos = f'x {r.x[k]:.2f} y {r.y[k]:.2f} ψ {math.degrees(r.yaw[k]):.0f}°' if k >= 0 else 'not started'
            if r.six_dof and k >= 0:
                pos += f' z {r.z[k]:.2f}'
            outcome = None if r.record is not None else (self.sim.outcome if self.sim else None)
            bad = int(it['viol'][:k + 1].sum()) if k >= 0 else 0
            rows.append(f"<p style='margin:2px 0'><b style='color:{it['color']}'>{r.name}</b>"
                        f"<br><span style='color:#8d8c85'>{vscene.meta_summary(r) or r.kind or ''}"
                        f"{' · ' + str(outcome) if outcome else ''}</span><br>{pos}"
                        f"{f'<br><span style=color:#ff5a52>{bad} samples in violation</span>' if bad else ''}</p>")
        self.info.setText(''.join(rows))
        lines = []
        for r in self.runs:
            for e in r.events:
                if e['t'] <= self.t + 1e-9:
                    lines.append((e['t'], f"{e['t']:7.1f}  {r.name[:14]:14}  [{e['lane']}] {self._event_text(e)}"))
        lines.sort()
        self.events_box.setPlainText('\n'.join(s for _, s in lines[-200:]))
        self.events_box.verticalScrollBar().setValue(self.events_box.verticalScrollBar().maximum())
        self._banner()

    def _banner(self):
        if self.sim is not None:
            text = (f'Live {self.sim.tier} {self.sim.domain} seed {self.sim.seed}, t = {self.sim.t:.1f} s'
                    f"{' · ' + self.sim.outcome if self.sim.outcome else ''}. Sliders act from now on; Rerun from t "
                    'restores the in-memory state at or before the playhead; Promote writes a crane run spec.')
            if self.recorded:
                text += ' Recorded runs beside it replay poses only.'
        else:
            text = ('Replay of recorded runs: a .ctraj holds poses only, so a rerun is from 0 only'
                    + (' (Rerun from 0 re-simulates from the config a run names).' if self.rerun_0.isEnabled()
                       else ' and none of these runs names a config to re-simulate.'))
        notes = sorted({n for r in self.runs for n in r.notes})
        if notes:
            text += ' Notes: ' + ', '.join(notes) + '.'
        self.banner.setText(text)

    # ---- 3-D -----------------------------------------------------------------------------------------------------

    def toggle_3d(self):
        on = self.three_btn.isChecked()
        if on and self.gl is None:
            try:
                import pyqtgraph.opengl as gl
                self.gl = gl.GLViewWidget()
                self.gl.setBackgroundColor(BG)
                grid = gl.GLGridItem()
                grid.setSize(200, 200)
                grid.setSpacing(5, 5)
                self.gl.addItem(grid)
                self.gl_items = [gl.GLLinePlotItem(width=2, antialias=True) for _ in self.runs]
                for item in self.gl_items:
                    self.gl.addItem(item)
                self.stack.addWidget(self.gl)
                pts = [np.column_stack(self._xy(r.x, r.y)) for r in self.runs if len(r.x)]
                if pts:
                    allp = np.vstack(pts)
                    lo, hi = allp.min(0), allp.max(0)
                    c = (lo + hi) / 2
                    self.gl.setCameraPosition(pos=pg.Vector(float(c[0]), float(c[1]), 0.0),
                                              distance=float(max(hi - lo)) * 1.3 + 5, elevation=35, azimuth=-60)
                    grid.translate(float(c[0]), float(c[1]), 0.0)
                self.gl_dots = gl.GLScatterPlotItem(size=9)
                self.gl.addItem(self.gl_dots)
            except Exception as error:  # noqa: BLE001  no OpenGL here (e.g. offscreen): stay 2-D and say so
                self._status(f'3-D view unavailable: {type(error).__name__}: {error}')
                self.three_btn.setChecked(False)
                return
        self.stack.setCurrentWidget(self.gl if on else self.map)
        if on:
            self._draw_3d()

    def _draw_3d(self):
        if len(self.gl_items) < len(self.runs):
            import pyqtgraph.opengl as gl
            for _ in range(len(self.runs) - len(self.gl_items)):
                self.gl_items.append(gl.GLLinePlotItem(width=2, antialias=True))
                self.gl.addItem(self.gl_items[-1])
        dots, colors = [], []
        for i, (r, item) in enumerate(zip(self.runs, self.gl_items)):
            k = r.index_at(self.t)
            if k < 1:
                continue
            x, y = self._xy(r.x[:k + 1], r.y[:k + 1])
            z = r.z[:k + 1] if r.z is not None else np.zeros(k + 1)
            col = pg.mkColor(RUN_COLORS[i % len(RUN_COLORS)])
            item.setData(pos=np.column_stack([x, y, z]), color=col)
            dots.append((float(x[-1]), float(y[-1]), float(z[-1])))
            colors.append(col.getRgbF())
        if dots:
            self.gl_dots.setData(pos=np.array(dots), color=np.array(colors))

    def show_3d(self):
        self.three_btn.setChecked(True)
        self.toggle_3d()
