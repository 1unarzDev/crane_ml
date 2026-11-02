"""Outputs: report JSON on disk, the one-line verdict (<= 160 chars, for the TUI agent panel, sim
templates and cluster job summaries), Markdown, and the viewer feed (insight anchors that jump a
replay scrubber to [run, t0, t1, channels])."""

from __future__ import annotations

import json
from pathlib import Path

MAX = 160


def _fmt(x, nd=2):
    if x is None:
        return '-'
    if isinstance(x, float):
        return f'{x:.{nd}f}'.rstrip('0').rstrip('.') if abs(x) < 1000 else f'{x:.0f}'
    return str(x)


def _clip(s):
    return s if len(s) <= MAX else s[:MAX - 1] + '~'


def oneline(report, label=''):
    k = report['kind']
    head = f'JUDGE {label + " " if label else ""}'
    ins = report.get('insights', [])
    if k == 'tiers':
        g = report['g4']
        worst = None
        for m, row in g['metrics'].items():
            if row.get('n'):
                score = abs(row.get('bias') or 0) / (row.get('band') or 1)
                if worst is None or score > worst[0]:
                    worst = (score, m, row)
        parts = [f"G4 {g['n_usable']}/{g['n_pairs']} usable"]
        if worst:
            _, m, row = worst
            parts.append(f"{m} bias {_fmt(row['bias'])} [{_fmt(row['bias_ci95'][0])},{_fmt(row['bias_ci95'][1])}]"
                         + (f" band {_fmt(row['band'])} {row.get('verdict')}" if row.get('band') is not None else ''))
        dom = report.get('split', {}).get('dominant') or {}
        if dom:
            parts.append('split ' + '/'.join(f'{k[:5]}{v}' for k, v in sorted(dom.items(), key=lambda kv: -kv[1])))
        dk = report.get('decisions', {}).get('kinds') or {}
        if dk.get('decision'):
            parts.append(f"{dk['decision']} decision-divergent")
        fm = g.get('failure_modes', {}).get('agreement')
        if fm is not None:
            parts.append(f'mode agree {fm:.0%}')
        if g.get('excluded'):
            parts.append('excl ' + ','.join(f'{v} {k}' for k, v in g['excluded'].items()))
        return _clip(head + ' | '.join(parts))
    if k == 'variants':
        bits = []
        for c in report['comparisons']:
            if c['estimate'] is None:
                continue
            bits.append(f"{c['metric']} {_fmt(c['estimate'])} [{_fmt(c['interval'][0])},{_fmt(c['interval'][1])}] "
                        f"{c['verdict']}")
        return _clip(head + f"n={report['n_pairs']} " + '; '.join(bits[:3]))
    if k == 'runs':
        oc = ', '.join(f'{v} {k}' for k, v in sorted(report['outcomes'].items(), key=lambda kv: -kv[1]))
        fails = sum(1 for i in ins if i['severity'] == 'fail')
        top = next((i['claim'] for i in ins if i['kind'] not in ('integrity.unscorable',) and not i['kind'].startswith('hint')), '')
        return _clip(head + f"{report['n']} runs: {oc} | {fails} fail-level | {top}")
    return _clip(head + k)


def markdown(report, title='Judge report'):
    lines = [f'# {title}', '', f"`{oneline(report)}`", '']
    if report['kind'] == 'tiers':
        g = report['g4']
        lines += ['## G4 gap (low - high)', '', '| metric | n | low | high | bias [95% CI] | sd | band | verdict |',
                  '|---|---|---|---|---|---|---|---|']
        for m, r in g['metrics'].items():
            if r.get('n'):
                lines.append(f"| {m} | {r['n']} | {_fmt(r['low_mean'])} | {_fmt(r['high_mean'])} | {_fmt(r['bias'])} "
                             f"[{_fmt(r['bias_ci95'][0])}, {_fmt(r['bias_ci95'][1])}] | {_fmt(r['sd'])} | "
                             f"{_fmt(r.get('band'))} | {r.get('verdict')} |")
        lines += ['', f"Excluded: {g['excluded'] or 'none'}; failure-mode agreement "
                  f"{_fmt(g['failure_modes']['agreement'])}", '']
        lines += ['| low outcome | high outcome | n |', '|---|---|---|']
        for k, v in g['failure_modes']['table'].items():
            a, b = k.split(' | ')
            lines.append(f'| {a} | {b} | {v} |')
        lines += ['', f"Split: {report.get('split')}", f"Decisions: {report.get('decisions')}", '']
        if report.get('g3'):
            lines += [f"## G3\n\n{json.dumps(report['g3'], default=str)}", '']
    if report['kind'] == 'variants':
        lines += ['| metric | role | n | estimate [CI] | p (Holm) | verdict |', '|---|---|---|---|---|---|']
        for c in report['comparisons']:
            lines.append(f"| {c['metric']} | {c['role']} | {c['n_units']} | {_fmt(c['estimate'])} "
                         f"[{_fmt(c['interval'][0])}, {_fmt(c['interval'][1])}] | {_fmt(c['p_holm'], 3)} | {c['verdict']} |")
    lines += ['', '## Insights', '']
    for i in report.get('insights', [])[:40]:
        lines.append(f"- **{i['severity']}** `{i['id']}` {i['claim']} _({i['basis']})_")
    return '\n'.join(lines) + '\n'


def feed(report):
    """Viewer anchors: {insight id: [[run_id, t0, t1, channels, label], ...]}."""
    out = {}
    for i in report.get('insights', []):
        spans = [[e.get('run_id'), e.get('t0'), e.get('t1'), e.get('channels'), e.get('label')]
                 for e in i.get('evidence', []) if isinstance(e, dict) and 'run_id' in e]
        if spans:
            out[i['id']] = spans
    return out


def write(report, out_dir, title='Judge report'):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'report.json').write_text(json.dumps(report, indent=1, sort_keys=True, default=str) + '\n')
    (out / 'report.md').write_text(markdown(report, title))
    (out / 'feed.json').write_text(json.dumps(feed(report), indent=1) + '\n')
    (out / 'verdict.txt').write_text(oneline(report) + '\n')
    return out
