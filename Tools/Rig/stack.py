"""The profile's stack launches the rig starts in the container: {cstart name: shell command}.

The rig owns the ROS-TCP endpoint (its port is per run), so it never starts the sensors launch.
Three profile formats:
- current: components: name device parts (no launch:); mhseals_testing resolves each through
  mhseals_nav's device.launch.py. That needs the container's install, so run_sim runs
  ``python3 stack.py profile.yaml`` there (this module's main) after the container is up.
- older: components: with launch: [package, file] and args: per component.
- oldest: bringup: args for mhseals_testing measurement.launch.py.
"""

import re
import shlex
import sys

SENSORS = 'sensors.launch.py'


def launch(package, file, args):
    return ' '.join(['ros2', 'launch', package, file] + [shlex.quote(f'{k}:={v}') for k, v in args.items()])


def name(key):
    """One cstart name per launch: each gets its own process group and a default SIGINT handler."""
    return f'stack_{re.sub(r"[^A-Za-z0-9_]", "_", key)}'


def needs_install(plant):
    """True for the current format: a started component without launch: (resolved in the container)."""
    components = plant.get('components') or {}
    return any(c.get('start') and not c.get('launch') for c in components.values())


def from_profile(plant):
    """{name: command} for the older formats, read from the profile alone."""
    components = plant.get('components')
    if components is not None:
        launches = {key: launch(*c['launch'], c.get('args') or {}) for key, c in components.items()
                    if c.get('start') and c.get('launch') and c['launch'][1] != SENSORS}
    else:
        bringup = dict(plant.get('bringup') or {}, start_sensors='false')
        launches = {'measurement': launch('mhseals_testing', 'measurement.launch.py', bringup)}
    return {name(key): cmd for key, cmd in launches.items()}


def from_components(items):
    """{name: command} from mhseals_testing Components: the started ones run here, never sensors."""
    out = {}
    for c in items:
        if not c.start or c.launch == SENSORS:
            continue
        if c.remote:
            raise ValueError(f'component {c.key} runs on {c.remote}, not in the rig container')
        out[name(c.key)] = shlex.join(c.argv)
    return out


def main(path):
    from mhseals_testing.harness.components import components
    from mhseals_testing.profiles import load_profile
    for key, cmd in from_components(components(load_profile(path))).items():
        print(key, cmd, sep='\t')


if __name__ == '__main__':
    main(sys.argv[1])
