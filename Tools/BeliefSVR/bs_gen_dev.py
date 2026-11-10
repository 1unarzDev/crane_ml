# SPDX-License-Identifier: GPL-3.0-or-later
"""Assemble results/DEV.md from generated tables (no hand-copied numbers in tables)."""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import bs_tables as T
RAW = os.path.join(HERE, "results", "raw")
rd = lambda p: open(p).read().strip()
tpl = open(os.path.join(HERE, "results", "DEV_template.md")).read()
subs = {
    "{{TUNING}}": T._tun(),
    "{{POL_V}}": T.policies("verification", "measured", 0.6, "COVER,FIXK1,FIXK2,SPRT@3.0,SPRT_DEFL@2.0,SPRT_DEFL_ALT@2.0,CI_THR@2.0,IND_BAYES@0.5,IND_BAYES@0.99,CORR@0.35,CORR@0.5,CORR_SAMEVIEW@0.35,CORR_PARTNER@0.35,ORACLE".split(",")),
    "{{POL_C}}": T.policies("coverage", "measured", 0.6, "COVER,FIXK1,FIXK2,SPRT@3.0,SPRT_DEFL@2.0,CI_THR@2.0,IND_BAYES@0.5,IND_BAYES@0.99,CORR@0.35,CORR@0.5,CORR_PARTNER@0.35,ORACLE".split(",")),
    "{{H_V}}": T.H(["verification"], [("H1", False), ("gain_vs_fixk@300", True), ("gain_vs_ind@300", True), ("gain_vs_cheap@300", True), ("cheap_over_best@300", True), ("viewclass@300", True), ("partner@300", True)],
                   ["H1 FR inflation at matched T", "J gain vs FIXK", "J gain vs IND_BAYES", "J gain vs deflated SPRT / CI", "cheap-fix J above best", "view-class choice", "partner"]),
    "{{H_C}}": T.H(["coverage"], [("H1", False), ("gain_vs_fixk@300", True), ("gain_vs_ind@300", True), ("gain_vs_cheap@300", True), ("cheap_over_best@300", True), ("viewclass@300", True), ("partner@300", True)],
                   ["H1 FR inflation at matched T", "J gain vs FIXK", "J gain vs IND_BAYES", "J gain vs deflated SPRT / CI", "cheap-fix J above best", "view-class choice", "partner"]),
    "{{EXTRA}}": rd(os.path.join(RAW, "extra.txt")),
    "{{CALIB}}": rd(os.path.join(RAW, "calib.md")),
    "{{TESTS}}": rd(os.path.join(HERE, "results", "tests.txt")).replace("\n", " ")[-120:] if os.path.exists(os.path.join(HERE, "results", "tests.txt")) else "not run",
}
for k, v in subs.items():
    tpl = tpl.replace(k, v)
open(os.path.join(HERE, "results", "DEV.md"), "w").write(tpl)
print("written", len(tpl))
