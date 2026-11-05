"""Help and usage docs match the code, stdlib unittest, no dotnet or Unity:
    python3 -m unittest discover -s Tools/RoboBoat/tests -p 'test_help.py'

- every course CLI subcommand and argument has help;
- every flag or subcommand the docs name exists: course_truth.py's in Docs/Course.md, coursegen's in
  cli/README.md and its usage text (Program.cs), generate.sh's, and the player's --crane-* options.
"""

import argparse
import os
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
TOOLS = HERE.parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(TOOLS))
import check_truth_subscribers  # noqa: E402
import course_truth  # noqa: E402
import coverage  # noqa: E402
import render  # noqa: E402
import validate  # noqa: E402

COURSE_MD = (ROOT / 'Docs' / 'Course.md').read_text()
CLI_README = (TOOLS / 'cli' / 'README.md').read_text()
PROGRAM = (TOOLS / 'cli' / 'Program.cs').read_text()
FLAG = r'(?<![^\s\[(|`"])(--?[a-z][a-z-]*)'


class Captured(Exception):
    pass


def parser_of(main):
    """The parser a script's main() builds, caught before it parses anything."""
    def grab(self, *args, **kwargs):
        raise Captured(self)
    with mock.patch.object(argparse.ArgumentParser, 'parse_args', grab):
        try:
            main([])
        except Captured as c:
            return c.args[0]
    raise AssertionError(f'{main.__module__}.main never parses arguments')


def walk(ap, name):
    """(name, action) for every argument and subcommand, recursively."""
    for a in ap._actions:
        if isinstance(a, argparse._HelpAction):
            continue
        if isinstance(a, argparse._SubParsersAction):
            helped = {c.dest: c.help for c in a._choices_actions}
            for cmd, sub in a.choices.items():
                yield f'{name} {cmd}', helped.get(cmd)
                yield from walk(sub, f'{name} {cmd}')
        else:
            yield f'{name} {"/".join(a.option_strings) or a.dest}', a.help


def options(ap):
    out = set()
    for a in ap._actions:
        out |= set(a.option_strings)
        if isinstance(a, argparse._SubParsersAction):
            for sub in a.choices.values():
                out |= options(sub)
    return out


class Help(unittest.TestCase):
    def test_every_argument_and_subcommand_has_help(self):
        for module in (course_truth, validate, render, coverage, check_truth_subscribers):
            missing = [n for n, h in walk(parser_of(module.main), module.__name__) if not h]
            self.assertEqual(missing, [], f'{module.__name__}: no help')

    def test_docstring_usage_names_real_options(self):
        for module in (validate, render, coverage):
            usage = [line for line in module.__doc__.splitlines() if f'{module.__name__}.py' in line]
            named = {f for line in usage for f in re.findall(FLAG, line)}
            self.assertLessEqual(named, options(parser_of(module.main)), module.__name__)


class Docs(unittest.TestCase):
    def test_course_md_names_every_course_truth_subcommand(self):
        ap = parser_of(course_truth.main)
        real = set(next(a for a in ap._actions if isinstance(a, argparse._SubParsersAction)).choices)
        for listed in re.findall(r'course_truth\.py ((?:\w+\|)+\w+)', COURSE_MD + course_truth.__doc__):
            self.assertEqual(set(listed.split('|')), real, listed)

    def test_coursegen_readme_usage_and_parser_agree(self):
        valued = re.search(r'Valued = \{([^}]*)\}', PROGRAM).group(1)
        switches = re.search(r'Switches = \{([^}]*)\}', PROGRAM).group(1)
        accepted = set(re.findall(r'"(--?[a-z-]+)"', valued + switches))
        usage = re.search(r'const string Usage =(.*?);', PROGRAM, re.S).group(1)
        self.assertEqual(set(re.findall(FLAG, usage)) - {'-h'}, accepted - {'-h'}, 'usage text vs accepted arguments')
        run = CLI_README[CLI_README.index('## Build and run'):CLI_README.index('Each run takes')]
        block = '\n'.join(line for line in run.splitlines() if 'coursegen.dll' in line or line.lstrip().startswith('#'))
        named = set(re.findall(FLAG, block)) - {'-c', '-p'}
        self.assertEqual(named, accepted - {'-h'}, 'cli/README.md run block vs accepted arguments')
        read = set(re.findall(r'Arg\(args, "(--[a-z-]+)"\)|args\.Contains\("(--?[a-z-]+)"\)|args\[i\] != "(--[a-z-]+)"', PROGRAM))
        self.assertLessEqual({f for t in read for f in t if f}, accepted, 'Program.cs reads an argument it rejects')

    def test_generate_sh_flags_in_course_md_exist(self):
        script = (TOOLS / 'generate.sh').read_text()
        arms = set(re.findall(FLAG, ' '.join(re.findall(r'^\s*([-|a-z]+)\)', script, re.M))))
        named = {f for m in re.findall(r'generate\.sh[^\n]*(?:\n  [^\n-][^\n]*)*', COURSE_MD) for f in re.findall(FLAG, m)}
        self.assertTrue(named)
        self.assertLessEqual(named, arms)

    def test_generate_sh_default_editor_is_project_version(self):
        """Without UNITY_EDITOR, generate.sh runs the Hub editor ProjectVersion.txt names (stub editor, no Unity)."""
        version = re.search(r'^m_EditorVersion: (\S+)', (ROOT / 'ProjectSettings' / 'ProjectVersion.txt').read_text(), re.M).group(1)
        with tempfile.TemporaryDirectory() as home:
            editor = Path(home, 'Unity', 'Hub', 'Editor', version, 'Editor', 'Unity')
            editor.parent.mkdir(parents=True)
            editor.write_text(f'#!/bin/sh\necho ran > "{home}/ran"\n')
            editor.chmod(0o755)
            env = {k: v for k, v in os.environ.items() if k != 'UNITY_EDITOR'} | {'HOME': home}
            subprocess.run(['bash', str(TOOLS / 'generate.sh'), '--family', 'f', '--seed', '0', '--out', f'{home}/t.json'],
                           env=env, capture_output=True, check=True)
            self.assertTrue(Path(home, 'ran').exists())

    def test_player_options_in_course_md_exist(self):
        code = ''.join(p.read_text() for p in (ROOT / 'Assets' / 'Scripts').rglob('*.cs'))
        named = set(re.findall(r'(--crane-[a-z-]+)', COURSE_MD))
        self.assertTrue(named)
        self.assertEqual([f for f in sorted(named) if f'"{f}"' not in code], [])


if __name__ == '__main__':
    unittest.main()
