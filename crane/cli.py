"""crane NOUN [VERB]: argparse built from the command registry (crane/commands.py), and the dispatch.

Stdlib only; each command's module is imported when it runs. Global flags (--json, --quiet, --dry-run) go before
the noun or after the verb, on commands that take them.
"""

import argparse
import re
import sys
import textwrap

from crane import commands as reg
from crane import output

TOP = """crane: simulators and experiments for our robots (boat, sub, drones, modular).

Words you will meet:
  tier         how detailed a simulation is. T0 abstract (no physics) and T1 dynamic (physics) are the
               light simulators: no graphics, many seeds a second. T2 unity is the full 3-D simulator, run
               in real time with the robot's own software. --tier takes T0|T1|T2 or abstract|dynamic|unity.
  ROS          the robot software framework; in T2 the same navigation software as on the boat drives.
  domain       a kind of robot world (boat, sub, drones, modular, toy); --domain picks one.
  seed         a number that fixes the random parts (course layout, noise): the same seed, the same run.
  run          one simulated episode for one seed, saved in its own folder.
  plant card   a robot's physics numbers (mass, drag, thrust...) measured on the water.
  bundle       the .tgz file astro saves after a day of testing; crane fit reads it.
  study        a planned experiment: several settings over many seeds, compared fairly.
  held-out     seeds kept back for the final test; crane refuses to run them unless you confirm.
  cluster      the shared pool of compute machines (ours and rented) that runs big studies.
  astro        the field-testing tool used on the water; crane reads the files it records."""

TOP_EXAMPLES = """examples:
  crane doctor                                            check this computer; says what to run next
  crane sim up                                            start the 3-D simulator and wait until it is ready
  crane run --domain toy --seeds 0:3                      simulate seeds 0, 1 and 2 in the light simulator
  crane run --domain boat --tier T1 --seeds 0:2           the boat's course with physics, seeds 0 and 1
  crane view --live --domain boat --tier T1 --seed 3      try settings live with sliders in a window
  crane help                                              every command; crane NOUN VERB --help for one

exit codes: 0 ok, 1 something failed (a FAIL verdict too), 2 a mistake in the command or config,
75 a wait ran out of time"""

FLAG_PARAMS, FLAG_OF, params_of = reg.FLAG_PARAMS, reg.FLAG_OF, reg.params_of


def first_sentence(text):
    return re.split(r'(?<!e\.g\.)(?<!i\.e\.)(?<=[.?])\s', text, maxsplit=1)[0].rstrip('.')


def _type(param):
    from crane import select

    def checked(fn):
        def parse(text):
            try:
                return fn(text)
            except select.SelectError as error:
                raise argparse.ArgumentTypeError(str(error)) from None
        parse.__name__ = param.type
        return parse
    return {'int': int, 'float': float, 'tier': checked(select.parse_tier),
            'env': checked(lambda t: select.parse_env(t) and t)}.get(param.type)


def _help_text(param):
    text = param.description
    if param.default not in (None, False, [], ()) and 'default' not in text:
        dot = text.endswith('.')
        text = text.rstrip('.') + f' (default {param.default})' + ('.' if dot else '')
    return text.replace('%', '%%')


def add_param(parser, param, groups):
    target = parser
    if param.group:
        if param.group not in groups:
            groups[param.group] = parser.add_mutually_exclusive_group(required=param.group in groups['__required__'])
        target = groups[param.group]
    kw = {'help': _help_text(param)}
    if param.positional:
        kw['nargs'] = param.nargs
        if param.nargs in ('?', '*'):
            kw['default'] = param.default
        if param.metavar:
            kw['metavar'] = param.metavar
        if param.choices:
            kw['choices'] = param.choices
        target.add_argument(param.dest or param.name.replace('-', '_'), **{k: v for k, v in kw.items() if v is not None
                                                                         or k == 'default'})
        return
    if param.dest:
        kw['dest'] = param.dest
    if param.type == 'bool':
        kw['action'] = 'store_true'
    else:
        if param.repeat:
            kw['action'] = 'append'
        if param.type == 'list':
            kw['nargs'] = param.nargs or '*'
        if _type(param):
            kw['type'] = _type(param)
        if param.choices and param.type not in ('tier',):
            kw['choices'] = param.choices
        if param.metavar:
            kw['metavar'] = param.metavar
        kw['default'] = list(param.default) if isinstance(param.default, (list, tuple)) else param.default
        if param.required:
            kw['required'] = True
    target.add_argument(param.name, **kw)


WIDTH = 100


def _wrap(text, indent=''):
    return '\n'.join(textwrap.wrap(text, WIDTH, initial_indent=indent, subsequent_indent=indent,
                                   break_on_hyphens=False, break_long_words=False))


def _description(command):
    parts = [command.summary]
    if command.today:
        parts.append(f'not yet available; today use {command.today}')
    if command.about:
        parts.append(f"what you'll see: {command.about}")
    if command.notes:
        parts.append(command.notes)
    return '\n\n'.join(_wrap(p) for p in parts)


def _epilog(command):
    lines = ['example:' if len(command.examples) == 1 else 'examples:', *(f'  {e}' for e in command.examples)]
    tech = [_wrap(command.details, '  ')] if command.details else []
    if command.raw:
        tech.append(f'  raw: {command.raw}')
    if command.json:
        tech.append(f'  --json schema: {command.json}')
    if tech:
        lines += ['', 'details:', *tech]
    return '\n'.join(lines)


def _usage(command):
    if not (command.passthrough or command.today):
        return None
    words = ' '.join(command.words)
    shown = ' '.join(f'[{p.name}{"" if p.type == "bool" else " " + (p.metavar or p.name.lstrip("-").upper())}]'
                     if not p.positional else (p.metavar or p.name.upper()) if p.nargs not in ('?', '*')
                     else f'[{p.metavar or p.name.upper()}]' for p in params_of(command))
    return f'crane {words} {shown} [ARGS...]'.replace('  ', ' ')


def _not_yet(command):
    return '(not yet) ' if command.today else ''


def leaf(sub, name, command, aliases=()):
    parser = sub.add_parser(name, aliases=list(aliases), help=_not_yet(command) + first_sentence(command.summary),
                            description=_description(command), epilog=_epilog(command), usage=_usage(command),
                            formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.set_defaults(_command=command)
    groups = {'__required__': set(command.groups)}
    for param in params_of(command):      # a passthrough's are shown in --help; its tool parses them
        add_param(parser, param, groups)
    if command.passthrough or command.today:
        parser.add_argument('rest', nargs=argparse.REMAINDER, help=argparse.SUPPRESS)
    return parser


def build_parser():
    ap = argparse.ArgumentParser(prog='crane', description=TOP, epilog=TOP_EXAMPLES,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--version', action='version', version=_version())
    sub = ap.add_subparsers(dest='noun', metavar='NOUN')
    for noun in reg.NOUNS:
        if not noun.verbs:
            leaf(sub, noun.name, reg.BY_WORDS[(noun.name,)], noun.aliases)
            continue
        np = sub.add_parser(noun.name, aliases=list(noun.aliases), description=noun.summary,
                            help=('(not yet) ' if all(c.today for c in noun.verbs) else '') + noun.summary,
                            epilog='examples:\n' + '\n'.join(f'  {c.examples[0]}' for c in noun.verbs),
                            formatter_class=argparse.RawDescriptionHelpFormatter,
                            usage=f'crane {noun.name} {{{",".join(c.verb for c in noun.verbs)}}} [ARGS...]')
        vs = np.add_subparsers(dest='verb', metavar='VERB', prog=f'crane {noun.name}')
        for command in noun.verbs:
            leaf(vs, command.verb, command)
    return ap


def _version():
    from crane import __version__
    return f'crane {__version__}'


def _leading_flags(argv):
    flags = set()
    while argv and argv[0] in FLAG_OF:
        flags.add(FLAG_OF[argv.pop(0)])
    return flags


def main(argv=None):
    argv = sys.argv[1:] if argv is None else list(argv)
    leading = _leading_flags(argv)
    if not argv or argv[0] in ('-h', '--help', '--version'):
        build_parser().parse_args(argv or ['--help'])
        return output.OK
    noun = reg.noun(argv[0])
    if noun is None:
        build_parser().parse_args(argv[:1])          # argparse names the choices, exit 2
        return output.USAGE
    if noun.verbs and (len(argv) < 2 or argv[1] in ('-h', '--help')):
        build_parser().parse_args([argv[0], '--help'])
        return output.OK
    command = reg.find(argv)
    if command is None and noun.owner:               # its tool's own verbs, usage and options-only form
        return reg.handler(noun.owner)(argv[1:])
    if command is None:
        build_parser().parse_args(argv[:2])          # an unknown verb: argparse names the choices, exit 2
        return output.USAGE
    words = len(command.words)
    rest = argv[words:]
    bad = leading - set(command.flags)
    if bad:
        flag = FLAG_PARAMS[sorted(bad)[0]].name
        return output.Output().usage(' '.join(command.words), f'{flag} is not supported here',
                                     f'crane {" ".join(command.words)} --help')
    if command.passthrough or command.today:
        if rest[:1] in (['-h'], ['--help']):
            build_parser().parse_args([*argv[:words], '--help'])
            return output.OK
        if command.today:
            print(reg.unavailable(command), file=sys.stderr)
            return output.USAGE
        rest += [FLAG_PARAMS[f].name for f in sorted(leading) if FLAG_PARAMS[f].name not in rest]
        return reg.handler(command)([*argv[1:words], *rest])
    args = build_parser().parse_args(argv)
    for flag in leading:
        setattr(args, flag, True)
    for flag in command.flags:
        if not hasattr(args, flag):
            setattr(args, flag, False)
    args.output = output.Output.of(args)
    return reg.handler(command)(args)


# ---- crane help ----------------------------------------------------------------------------------------------

def help_main(args):
    out = args.output
    if args.check or args.write:
        return reg.check(write=args.write, out=out)
    topic = list(args.topic or [])
    if topic:
        command = reg.find(topic)
        noun = reg.noun(topic[0])
        if noun is None or (len(topic) > 1 and command is None):
            return out.usage('help', f'no command {" ".join(topic)!r}', 'crane help')
        shown = [command] if len(topic) > 1 or not noun.verbs else noun.verbs
    else:
        shown = list(reg.COMMANDS)
    if out.doc({'schema': 'crane.help/1', 'commands': [reg.as_dict(c) for c in shown]}):
        return output.OK
    if topic and len(shown) == 1:
        build_parser().parse_args([*shown[0].words, '--help'])
        return output.OK
    rows = [(c.name, _not_yet(c) + first_sentence(c.summary)) for c in shown]
    width = max(len(r[0]) for r in rows)
    for name, summary in rows:
        print(f'  {name:<{width}}  {summary}')
    if topic:
        print('examples:')
        for c in shown:
            print(f'  {c.examples[0]}')
    out.next(f'crane {topic[0]} VERB --help' if topic else 'crane NOUN [VERB] --help')
    return output.OK


if __name__ == '__main__':
    sys.exit(main())
