"""The field-to-sim path on a synthetic astro session: the T1 twin runs astro's speed test like the boat would,
and its odometry, thruster PWM and /events lines go into an MCAP written here; the result files, state and
log.txt go into a bundle beside it.

- bundle + MCAP -> crane.run/1 (frame, clock, resampled trajectory, events from /events and log.txt once);
- bundle -> crane fit: the version-1 characterization is mapped (or abstains naming what v2 adds), the
  generic software key is provenance, every hold becomes evidence, a card is fitted with provenance;
- the same test in sim -> crane compare --field: the same plant gives zero deltas, T0 does not; card factorials
  run through the manifest; runs on different frames or clocks are refused.
"""

import json
import math
import struct
import sys
import tarfile
from pathlib import Path

import numpy as np
import pytest
import yaml

from crane import cli, compare, field, fit, paths, record

CHAR = paths.ROOT / 'Tools' / 'Characterization'
sys.path.insert(0, str(CHAR))
import astro_tests as A  # noqa: E402
import gate as G  # noqa: E402

PROFILE = CHAR / 'profiles' / 'characterized.json'
WALL0 = 1795540000.0                     # the session's wall clock at sim t = 0
DAY = '2026-11-24'
TESTS = {'axes': ['surge', 'sway', 'yaw'], 'rotational': 'yaw',
         'spaces': {'small': {'amp': 0.3, 'levels': [0.2, 0.35, 0.5]}},
         'tests': {'speed-surge': {'type': 'speed', 'axis': 'surge', 'baseline_s': 2, 'settle_s': 5, 'measure_s': 3,
                                   'coast_s': 4, 'apply': {'key': 'characterization', 'version': 1}},
                   'still': {'type': 'still', 'seconds': 5}},
         'groups': {'all': ['still', 'speed-surge']}}


# ---- a minimal MCAP writer (uncompressed: what mcap_lite reads) --------------------------------------------

class CDR:
    def __init__(self):
        self.b = bytearray(b'\x00\x01\x00\x00')

    def put(self, fmt, *v):
        size = struct.calcsize('<' + fmt[0])
        self.b += b'\0' * ((-(len(self.b) - 4)) % size)
        self.b += struct.pack('<' + fmt, *v)

    def string(self, s):
        raw = s.encode() + b'\0'
        self.put('I', len(raw))
        self.b += raw


def odom_msg(stamp, frame, x, y, yaw, vx, vy, wz):
    c = CDR()
    c.put('i', int(stamp)), c.put('I', int(round((stamp % 1) * 1e9)) % 1000000000)
    c.string(frame), c.string('base_link')
    c.put('ddd', x, y, 0.0), c.put('dddd', 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    c.put('d' * 36, *[0.0] * 36), c.put('ddd', vx, vy, 0.0), c.put('ddd', 0.0, 0.0, wz), c.put('d' * 36, *[0.0] * 36)
    return bytes(c.b)


def pwm_msg(values):
    c = CDR()
    c.put('I', 0), c.put('I', 0), c.put('I', len(values)), c.put('H' * len(values), *[int(v) for v in values])
    return bytes(c.b)


def string_msg(text):
    c = CDR()
    c.string(text)
    return bytes(c.b)


def write_mcap(path, messages):
    """messages: [(topic, log_time_s, cdr bytes)]."""
    def record(op, body):
        return bytes([op]) + struct.pack('<Q', len(body)) + body
    out = bytearray(b'\x89MCAP0\r\n')
    ids = {}
    for topic, _, _ in messages:
        if topic not in ids:
            ids[topic] = len(ids) + 1
            t = topic.encode()
            out += record(4, struct.pack('<HH', ids[topic], 0) + struct.pack('<I', len(t)) + t
                          + struct.pack('<I', 3) + b'cdr' + struct.pack('<I', 0))
    for seq, (topic, t, data) in enumerate(sorted(messages, key=lambda m: m[1])):
        ns = int(round(t * 1e9))
        out += record(5, struct.pack('<HIQQ', ids[topic], seq, ns, ns) + data)
    out += record(15, b'\0' * 20) + b'\x89MCAP0\r\n'
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_bytes(bytes(out))


# ---- the synthetic session --------------------------------------------------------------------------------

def _wall(t):
    return WALL0 + t


def session_bundle(tmp, *, frame='odom', with_bag=True, seed=0):
    """(bundle.tgz, vessel, results, session): still + speed-surge on the T1 twin, as astro would bundle them."""
    prof = json.loads(PROFILE.read_text())
    spec = A.Spec(TESTS)
    vessel = A.ProfileVessel(prof)
    session = A.Session(spec, vessel)
    results = [session.run(name, space='small') for name in ('still', 'speed-surge')]
    rng = np.random.default_rng(seed)
    root = Path(tmp) / 'src'
    (root / 'state').mkdir(parents=True)
    (root / DAY / 'results').mkdir(parents=True)
    char = dict(session.state['characterization'], source='astro speed tests on the lake')
    state = {'characterization': char, 'channel_map': [1, 2, 3, 4], 'reversed': [False] * 4,
             'thruster_matrix': [v for row in G.DEFAULT_MATRIX for v in row]}
    (root / 'state' / 'hardware.yaml').write_text(yaml.safe_dump(state))
    (root / 'state' / 'tests.yaml').write_text(yaml.safe_dump(TESTS))
    lines = []
    for i, r in enumerate(results):
        r = json.loads(json.dumps(r))
        r['software'] = {'roboboat_ws': 'a' * 40}
        for h in r.get('holds') or []:
            h['start'], h['end'] = _wall(h['start']), _wall(h['end'])
            s = h['samples']
            s['t'] = [round(_wall(t), 4) for t in s['t']]
            for k, sd in (('vx', 0.01), ('vy', 0.01), ('wz', 0.005)):
                s[k] = [round(v + float(rng.normal(0, sd)), 4) for v in s[k]]
        (root / DAY / 'results' / f"{r['test']}-17000{i}.yaml").write_text(yaml.safe_dump(r, sort_keys=False))
    for e in session.events:
        hms = (np.datetime64(int(_wall(e['t'])), 's') + np.timedelta64(1, 'h')).astype(object).strftime('%H:%M:%S')
        lines.append((e['t'], f'{hms}  {e["text"]}'))      # log.txt is local time, here UTC+1
    log = [text for _, text in lines] + ['23:59:59  note    only in log.txt']
    (root / DAY / 'log.txt').write_text('\n'.join(log) + '\n')
    if with_bag:
        msgs = []
        mix = np.array(A.mixer_rows(prof))
        for t, vx, vy, wz, yaw, x, y in vessel.rows:
            msgs.append(('/odom/local', _wall(t) + 0.002, odom_msg(_wall(t), frame, x, y, yaw, vx, vy, wz)))
        for k in range(0, len(vessel.rows), 2):          # the effort the vessel held, as physical PWM at 25 Hz
            t = vessel.rows[k][0]
            msgs.append(('/thrusters/pwm', _wall(t), pwm_msg([1500 + 400 * q for q in mix @ _effort_at(results, t)])))
        for t, text in lines:
            msgs.append(('/events', _wall(t) + 0.01, string_msg(text)))
        write_mcap(root / DAY / 'bags' / '170000' / '170000_0.mcap', msgs)
    tgz = Path(tmp) / 'bundle-170000.tgz'
    with tarfile.open(tgz, 'w:gz') as tar:
        for p in sorted(root.rglob('*')):
            tar.add(p, arcname=str(p.relative_to(root)), recursive=False)
    return tgz, vessel, results, session


def _effort_at(results, t):
    for r in results:
        for h in r.get('holds') or []:
            if h['start'] <= t < h['end']:
                return np.array(h['command']['efforts'])
    return np.zeros(3)


@pytest.fixture(scope='module')
def bundle(tmp_path_factory):
    return session_bundle(tmp_path_factory.mktemp('session'))


# ---- tests ----------------------------------------------------------------------------------------------

def test_field_import_puts_bag_and_log_on_one_clock_and_frame(bundle, tmp_path, capsys):
    tgz, vessel, _, session = bundle
    assert cli.main(['field', 'import', str(tgz), '--out', str(tmp_path / 'run')]) == 0
    rec = record.read_run(tmp_path / 'run')
    assert rec.meta['kind'] == 'field' and rec.frame == {'world': 'enu', 'origin': 'odom', 'body': 'flu'}
    assert rec.clock['source'] == 'wall' and rec.meta['timing']['t_src'] == 'header.stamp'
    assert rec.clock['t0'] == pytest.approx(_wall(vessel.rows[0][0]))
    # the trajectory is the odometry resampled on header stamps: back on the vessel's rows within a cm
    tr = rec.traj
    times = np.asarray(tr.times())
    rows = np.array(vessel.rows)
    t_rows = rows[:, 0] - rows[0, 0]
    k = np.searchsorted(t_rows, times[10:-10:37])
    assert np.allclose(np.asarray(tr.columns['boat.x'])[10:-10:37], rows[k, 5], atol=0.02)
    # /events lines and log.txt lines: once each, on the record clock; the extra log line placed by the tz offset
    evs = rec.events
    starts = [e for e in evs if 'start speed-surge' in e['text']]
    assert len(starts) == 1 and starts[0]['source'] == '/events'
    sent = next(e['t'] for e in session.events if 'start speed-surge' in e['text'])
    assert starts[0]['t'] == pytest.approx(sent - vessel.rows[0][0], abs=0.02)
    only_log = [e for e in evs if e['source'] == 'log.txt']
    assert [e['log_kind'] for e in only_log] == ['note']
    assert rec.meta['provenance']['events_clock']['tz_from_events_s'] == pytest.approx(-3600, abs=1)
    capsys.readouterr()


def test_unknown_odometry_frame_is_refused_unless_named(tmp_path):
    tgz = session_bundle(tmp_path / 's', frame='utm_local')[0]
    with pytest.raises(field.FieldError, match='utm_local'):
        field.import_any(tgz, tmp_path / 'a')
    run = field.import_any(tgz, tmp_path / 'b', frames={'utm_local': 'ned'})[0]
    assert run.frame == {'world': 'ned', 'origin': 'utm_local', 'body': 'frd'}


def test_fit_maps_version_1_names_software_and_fits_a_card_from_every_hold(bundle, tmp_path):
    tgz, _, results, _ = bundle
    summary = fit.run_fit([tgz], tmp_path / 'fit', log=lambda *a: None)
    gate = summary['gate']
    assert gate['mapping']['mapped'] and set(gate['mapping']['t95']) == {'surge'}
    missing = [m for reason, rows in gate['abstained'].items() if reason == 'PROVENANCE_MISSING'
               for _, what in rows for m in what]
    assert not any('version' in m or 'software' in m for m in missing), missing
    assert summary['software'] == {'roboboat_ws': 'a' * 40}
    n_holds = sum(len(r.get('holds') or []) for r in results)
    assert summary['evidence']['n_segments'] == n_holds == 6
    card = summary['card']
    doc = json.loads(next((tmp_path / 'fit' / 'cards' / 'cards').glob(card['candidate'][:12] + '*'))
                     .joinpath('card.json').read_text())
    prov = doc['provenance']['evidence']['bundles'][0]
    assert prov['sha256'] and prov['results'] and prov['software'] == {'roboboat_ws': 'a' * 40}
    assert doc['status'] == 'fitted' and card['validation']['sessions'] == 1 and 'promotion' not in card['validation']
    assert doc['blocks']['dyn.surge']['n_windows'] > 0 and doc['parent'] == card['prior']
    # a version 1 that is not signed d1/d2 curves cannot be mapped, and says what version 2 adds
    char, m = G.characterization_v2({'version': 1, 'speed_curves': {'surge': {'d1': 1, 'd2': 1}}}, {})
    assert char['version'] == 1 and 'version 2 adds' in m['missing'] and 'wave_floor' in m['missing']


def test_same_tests_in_sim_compare_to_the_field_key_by_key(bundle, tmp_path, capsys):
    tgz = bundle[0]
    common = ['--domain', 'boat', '--astro-test', 'all', '--space', 'small', '--tests', str(tgz)]
    assert cli.main(['run', '--tier', 'T1', *common, '--card', str(PROFILE), '--out', str(tmp_path / 't1')]) == 0
    assert cli.main(['run', '--tier', 'T0', *common, '--card', str(PROFILE), '--out', str(tmp_path / 't0')]) == 0
    rec = record.read_run(tmp_path / 't1' / 'characterized' / 'speed-surge' / 's0')
    assert rec.meta['tier'] == 'T1' and rec.meta['kind'] == 'dynamic' and rec.metrics['type'] == 'speed'
    res = compare.compare_field(tgz, tmp_path / 't1')
    rows = {r['metric']: r for r in res['speed-surge']['characterized']['rows']}
    # the field session ran the same twin: every number of the result agrees, and t95 has a CI over its holds
    assert {'speed_curves.surge+.d1', 'motion_limits.velocity.surge', 't95.surge', 'measured.speed.surge+@0.5'} <= set(rows)
    assert all(abs(r['delta']) < 1e-9 for r in rows.values())
    assert rows['measured.t95']['ci'] and rows['measured.t95']['ci_over'] == 'field samples'
    assert rows['motion_limits.velocity.surge']['unit'] == 'm/s'
    t0 = {r['metric']: r for r in compare.compare_field(tgz, tmp_path / 't0')['speed-surge']['characterized']['rows']}
    assert abs(t0['t95.surge']['delta']) > 0.05
    # tier agreement and gap over the same runs, as a table and JSON
    assert cli.main(['compare', str(tmp_path / 't0'), str(tmp_path / 't1'), '--tiers', '--metric', 't95.surge',
                     '--out', str(tmp_path / 'cmp')]) == 0
    out = json.loads((tmp_path / 'cmp' / 'compare.json').read_text())['result']
    assert out['tiers'] == {'low': ['T0'], 'high': ['T1']} and out['gap']['n_usable'] == 2
    assert '| t95.surge |' in capsys.readouterr().out


def test_factorial_variants_run_through_the_manifest(tmp_path, capsys):
    spec = {'measured': str(PROFILE), 'prior': str(CHAR / 'profiles' / 'ideal.json'),
            'properties': {'surge': ['m11', 'd1_u', 'd2_u'], 'actuation': ['delay_s']},
            'hypotheses': {'actuation': {'slow': {'delay_s': 0.4}}},
            'run': {'tier': 'T1', 'astro_test': 'speed-surge', 'space': 'small'}}
    (tmp_path / 'f.yaml').write_text(yaml.safe_dump(spec))
    man = fit.factorial(tmp_path / 'f.yaml', tmp_path / 'fac', log=lambda *a: None)
    assert len(man['variants']) == 2 * 3
    from plantcard.marine.fossen3 import profile_params
    prior = profile_params(json.loads((CHAR / 'profiles' / 'ideal.json').read_text()))
    v = next(v for v in man['variants'] if v['levels'] == {'surge': 'prior', 'actuation': 'slow'})
    prof = json.loads((tmp_path / 'fac' / v['profile']).read_text())
    assert prof['surge']['d1Pos'] == pytest.approx(prior['d1_u'])
    assert prof['thrusters'][0]['deadTime'] == pytest.approx(0.4)
    assert prof['provenance']['factorial']['levels'] == v['levels']
    tests = tmp_path / 'tests.yaml'
    tests.write_text(yaml.safe_dump(TESTS))
    assert cli.main(['run', '--domain', 'boat', '--tier', 'T1', '--astro-test', 'speed-surge', '--space', 'small',
                     '--tests', str(tests), '--card', str(tmp_path / 'fac' / 'manifest.json'),
                     '--out', str(tmp_path / 'runs')]) == 0
    assert sorted(p.name for p in (tmp_path / 'runs').iterdir()) == sorted(x['name'] for x in man['variants'])
    capsys.readouterr()


def test_compare_refuses_runs_it_cannot_put_on_one_frame_and_clock(bundle, tmp_path, capsys):
    tgz = bundle[0]
    field.import_any(tgz, tmp_path / 'a' / 'x')
    tests = tmp_path / 'tests.yaml'
    tests.write_text(yaml.safe_dump(TESTS))
    cli.main(['run', '--domain', 'boat', '--tier', 'T1', '--astro-test', 'still', '--tests', str(tests),
              '--out', str(tmp_path / 'sim')])
    sim = next((tmp_path / 'sim').rglob('run.json')).parent
    res = compare.compare_ab(tmp_path / 'a' / 'x', sim)
    assert 'origins differ' in res['pairs'][0]['pose_refused']
    capsys.readouterr()


def test_compare_bounds_pass_fail_gate_and_ci_rule(bundle, tmp_path, capsys):
    tgz = bundle[0]
    common = ['--domain', 'boat', '--astro-test', 'all', '--space', 'small', '--tests', str(tgz), '--card', str(PROFILE)]
    for tier in ('T0', 'T1'):
        assert cli.main(['run', '--tier', tier, *common, '--out', str(tmp_path / tier)]) == 0
    capsys.readouterr()
    run = ['compare', '--field', str(tgz)]

    # the same plant: every bounded row passes with the shipped boat bounds; the gate is not checked and does not fail
    assert cli.main([*run, str(tmp_path / 'T1'), '--bounds', 'file', '--fit', str(tmp_path / 'nofit')]) == 0
    out = capsys.readouterr().out
    assert 'gate: not checked' in out and 'verdict: pass (' in out and 'bounds)' in out and 'next:' in out
    assert 'measured.speed.surge+@0.5' in out and ' n/a' in out          # t95.surge has no rule

    # JSON: crane.compare/1, its result holds the rows plus the verdict
    assert cli.main([*run, str(tmp_path / 'T1'), '--bounds', 'file', '--json', '--fit', str(tmp_path / 'nofit')]) == 0
    doc = json.loads(capsys.readouterr().out)
    assert doc['schema'] == 'crane.compare/1' and doc['kind'] == 'bounds'
    doc = doc['result']
    assert doc['verdict']['pass'] and doc['verdict']['bounds'] == 7 and doc['verdict']['gate'] == 'not checked'
    assert {'metric', 'unit', 'field', 'sim', 'delta', 'result', 'bound', 'rule'} <= set(doc['rows'][0])
    assert {r['result'] for r in doc['rows']} == {'pass', 'n/a'}

    # T0 against tight bounds: FAIL lines name the metric, the delta, the bound and a fix; exit 1
    tight = tmp_path / 'tight.yaml'
    tight.write_text(yaml.safe_dump({'gate': {'require': 'identified'}, 'rules': [
        {'metric': 'measured.speed.*', 'rel': 0.001, 'unit': 'm/s'},
        {'metric': 'measured.t95', 'rel': 0.001, 'ci_zero_ok': False, 'unit': 's'}]}))
    assert cli.main([*run, str(tmp_path / 'T0'), '--bounds', str(tight), '--fit', str(tmp_path / 'nofit')]) == 1
    out = capsys.readouterr().out
    assert 'FAIL speed-surge measured.t95: delta -' in out and 'outside +-' in out and 'fix: refit: crane fit' in out
    assert 'verdict: FAIL (' in out and 'outside bounds' in out

    # a gate that is not identified fails an otherwise passing compare
    fit_dir = tmp_path / 'fit'
    fit_dir.mkdir()
    (fit_dir / 'fit.json').write_text(json.dumps({'gate': {'parameters': [
        {'name': 'surge+', 'status': 'identified', 'reason': ''}, {'name': 'yaw+', 'status': 'prior', 'reason': 'x'},
        {'name': 'mass', 'status': 'prior', 'reason': 'no weigh-in'}]}}))
    assert cli.main([*run, str(tmp_path / 'T1'), '--bounds', 'file', '--fit', str(fit_dir)]) == 1
    assert 'gate: yaw+ not identified)' in capsys.readouterr().out
    (fit_dir / 'fit.json').write_text(json.dumps({'gate': {'parameters': [
        {'name': 'surge+', 'status': 'identified', 'reason': ''}]}}))
    assert cli.main([*run, str(tmp_path / 'T1'), '--bounds', 'file', '--fit', str(fit_dir)]) == 0
    assert 'gate: all identified' in capsys.readouterr().out


def test_compare_bounds_usage_errors_and_ci_zero_rule(bundle, tmp_path, capsys):
    tgz = bundle[0]
    assert cli.main(['compare', '--field', str(tgz), str(tmp_path), '--bounds', 'card']) == 2
    assert 'not yet available' in capsys.readouterr().err
    assert cli.main(['compare', '--field', str(tgz), str(tmp_path), '--bounds', 'file', '--domain', 'nowhere']) == 2
    err = capsys.readouterr().err
    assert err.count('\n') == 1 and 'config/fidelity/nowhere.yaml' in err
    assert cli.main(['compare', str(tmp_path), str(tmp_path), '--bounds', 'file']) == 2
    capsys.readouterr()
    row = {'metric': 'measured.t95', 'field': 2.0, 'sim': 3.0, 'delta': 1.0, 'unit': 's', 'ci': [-0.2, 2.0]}
    rule = {'metric': 'measured.t95', 'rel': 0.25, 'ci_zero_ok': True}
    assert compare.judge_row(row, rule)['result'] == 'pass'                       # |delta| 1.0 > 0.5, but the CI has 0
    assert compare.judge_row(row, {**rule, 'ci_zero_ok': False})['result'] == 'FAIL'
    assert compare.judge_row({**row, 'ci': [0.1, 2.0]}, rule)['result'] == 'FAIL'
    assert compare.judge_row({**row, 'ci': None}, rule)['result'] == 'FAIL'
    assert compare.judge_row(row, None)['result'] == 'n/a'
    cross = {'metric': 'measured.cross.*', 'abs': 0.02}
    assert compare.judge_row({**row, 'delta': 0.019}, cross)['result'] == 'pass'
    assert compare.judge_row({**row, 'delta': -0.021, 'ci': None}, cross)['result'] == 'FAIL'
