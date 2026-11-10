"""Driver: threshold / pessimism-margin analysis of stored DecisionCal runs (M2, M3; then M4 as a separate section).
    nice -n 19 python3 -I analysis/analyze_threshold.py [--m4] > analysis/output_threshold.txt
Read-only on ~/.cache/decisioncal; numpy only; one process."""
import sys, os
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from thr_common import *
import thr_q1, thr_q1b, thr_q1c, thr_q2, thr_q3_6

def out(s=''): print(s, flush=True)

def section_m23():
    out('#' * 100); out('# PART A: M2 (m2-20261112) + M3 (m3-20261113)'); out('#' * 100)
    rows = {'m2': load('m2-20261112'), 'm3': load('m3-20261113')}
    # Q1 pooled (fitted + assumed arms; rigid = 0)
    for lib in ('v2', 'v3'):
        out(f'===== Q1 pooled, library {lib}: all arms that carry a delay (rigid=0, rigid+delay, joint, rigid@d*), M2+M3 boats; excludes rigid+lag')
        for st in ('timing', 'transient'):
            R = thr_q1.pooled(rows, st, lib)
            out(f'-- {st}: {len({r["id"] for r in R})} boats, {len(R)} picks')
            thr_q1.report_threshold(R, st, out); thr_q1.harm_curve(R, st, out)
        out(f'-- fitted-delay arms only (rigid+delay, joint), timing boats, same bins (endogenous: fitted delay co-varies with the boat):')
        R = thr_q1.pooled(rows, 'timing', lib, arms=('rigid+delay', 'joint')); thr_q1.harm_curve(R, 'timing fitted-only', out)
    thr_q1b.main(out, 'm3-20261113')
    for lib in ('v2', 'v3'):
        out(f'===== Q1c ({lib}) M3 timing, assumed-delay arms only (exogenous)')
        thr_q1c.fail_curve(thr_q1b.fixed_recs(rows['m3'], 'timing', lib), f'M3 timing {lib}', out)
        thr_q1c.delta_terciles(thr_q1b.fixed_recs(rows['m3'], 'timing', lib), f'M3 timing {lib}', out)
    thr_q2.main(out)
    thr_q3_6.main(out, 'm3-20261113', 'M3')
    out('#' * 100); out('# PART A2: M2 only (Q3 optimism, Q4/Q5 not applicable: no assumed levels)'); out('#' * 100)
    thr_q3_6.q3(rows['m2'], 'M2', out)

def section_m4():
    out('#' * 100); out('# PART B: M4 (m4-20261114), pre-registered run, analysed with the same code. SEPARATE from Part A.'); out('#' * 100)
    run = 'm4-20261114'; rows = load(run)
    R4 = {'m4': rows}
    for lib in ('v2', 'v3'):
        out(f'===== M4 Q1 pooled, library {lib}: arms with a delay (rigid=0, rigid+delay, joint, rigid@d0.35/0.47/0.62)')
        for st in ('timing', 'transient', 'steady'):
            R = thr_q1.pooled(R4, st, lib)
            out(f'-- {st}: {len({r["id"] for r in R})} boats, {len(R)} picks')
            if st != 'steady': thr_q1.report_threshold(R, st, out)
            thr_q1.harm_curve(R, st, out, edges=(-1, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.45, 0.6, 1))
    # fixed-arm tables
    for lib in ('v2', 'v3'):
        out(f'===== M4 Q1b library {lib}: assumed-delay arms only')
        Rt = thr_q1b.fixed_recs(rows, 'timing', lib)
        thr_q1b.L_table(Rt, 'M4 timing', out)
        for how in ('reg10', 'fail'):
            thr_q1b.report_maxscore(Rt, 'M4 timing', out, how); thr_q1b.cv_models(Rt, how, 'M4 timing', out)
        Rs = thr_q1b.fixed_recs(rows, 'steady', lib)
        thr_q1b.L_table(Rs, 'M4 steady', out, edges=(0, 0.1, 1.0))
        Rr = thr_q1b.fixed_recs(rows, 'transient', lib)
        thr_q1b.L_table(Rr, 'M4 transient', out, edges=(0, 0.1, 1.0))
        out(f'--- M3+M4 timing boats pooled (100 boats; levels 0,.16,.25,.35 on M3 boats, 0,.35,.47,.62 on M4 boats): fixed arms only')
        Rp = thr_q1b.fixed_recs(load('m3-20261113') + rows, 'timing', lib)
        for how in ('reg10', 'fail'):
            thr_q1b.report_maxscore(Rp, 'M3+M4 timing', out, how); thr_q1b.cv_models(Rp, how, 'M3+M4 timing', out)
        thr_q1b.covariate_shifts(Rp, 'M3+M4 timing', out)
    for lib in ('v2', 'v3'):
        out(f'===== M4 Q1c ({lib})')
        thr_q1c.fail_curve(thr_q1b.fixed_recs(load('m3-20261113') + rows, 'timing', lib), f'M3+M4 timing assumed-delay arms {lib}', out)
        thr_q1c.fail_curve(thr_q1b.fixed_recs(rows, 'timing', lib), f'M4 timing assumed-delay arms {lib}', out)
        thr_q1c.delta_terciles(thr_q1b.fixed_recs(load('m3-20261113') + rows, 'timing', lib), f'M3+M4 timing {lib}', out)
        out('  over-estimation cost (hinge fit of capped regret on excess):')
        for st in ('steady', 'transient', 'timing'):
            thr_q1c.hinge_report(thr_q1.pooled(R4, st, lib), f'M4 {st} {lib} all carried-delay arms', out, min_ex=-0.05)
            thr_q1c.hinge_report(thr_q1b.fixed_recs(rows, st, lib), f'M4 {st} {lib} assumed-delay arms only', out, min_ex=-0.05)
    # Q2 pick params
    for lib in ('v2', 'v3'):
        out(f'===== M4 Q2 library {lib}')
        for st in ('timing', 'transient', 'steady'):
            arms = ['rigid', 'rigid@d0.35', 'rigid@d0.47', 'rigid@d0.62', 'rigid+delay', 'joint', 'ORACLE']
            out(f'--- {st}'); thr_q2.pick_means(rows, st, lib, arms, out)
        thr_q2.pick_shift(rows, 'timing', lib, 'rigid@d0.47', 'rigid', 'M4 timing, @0.47 minus rigid', out)
        thr_q2.pick_shift(rows, 'steady', lib, 'rigid@d0.47', 'rigid', 'M4 steady, @0.47 minus rigid', out)
        thr_q2.pick_shift(rows, 'steady', lib, 'rigid@d0.62', 'rigid', 'M4 steady, @0.62 minus rigid', out)
        thr_q2.pick_shift(rows, 'steady', lib, 'rigid+delay', 'rigid', 'M4 steady, fitted delay minus rigid', out)
        thr_q2.pick_shift(rows, 'timing', lib, 'rigid@d0.62', 'rigid@d0.47', 'M4 timing, @0.62 minus @0.47', out)
        thr_q2.pick_shift(rows, 'transient', lib, 'rigid@d0.62', 'rigid@d0.47', 'M4 transient, @0.62 minus @0.47', out)
    # Q3-Q6
    thr_q3_6.q3(rows, 'M4', out)
    for lib in ('v2', 'v3'):
        out(f'##### Q4 dose response (M4) {lib}')
        for gd in ('reg10', 'dock'):
            thr_q3_6.q4(rows, 'M4', out, 'timing', ('L1', 'dead_time', 'odom_latency', 'tau_up', 'slew', 'gamma'), lib, gd)
            thr_q3_6.q4(rows, 'M4', out, 'transient', ('tau_up', 'tau_down', 'rev_hold', 'mass'), lib, gd)
    thr_q3_6.q5(rows, 'M4', out, strata=('timing', 'transient', 'steady'))
    for a in ('rigid@d0.35', 'rigid@d0.47'):
        thr_q3_6.q5_fail(rows, 'M4', out, a, strata=('timing', 'transient'))
    thr_q3_6.q6(rows, 'M4', out, strata=('timing', 'transient', 'steady'))

if __name__ == '__main__':
    if '--m4-only' not in sys.argv: section_m23()
    if '--m4' in sys.argv or '--m4-only' in sys.argv: section_m4()
