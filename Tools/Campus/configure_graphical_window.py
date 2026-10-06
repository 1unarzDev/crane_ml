#!/usr/bin/env python3
"""Keep this campus player updating on hidden Hyprland workspaces; no config edits."""
import argparse, json, os, pathlib, re, shutil, subprocess, time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', type=pathlib.Path, required=True)
    args = parser.parse_args()
    args.run.mkdir(parents=True, exist_ok=True)
    report = {'schema': 'crane-campus-desktop-render-v1', 'state': 'SKIPPED',
              'scope': 'current campus player window only', 'configFilesModified': False}
    try:
        if not os.environ.get('HYPRLAND_INSTANCE_SIGNATURE') or not shutil.which('hyprctl'):
            report['reason'] = 'Hyprland unavailable; keep the graphical window visible for QA.'
            return
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            clients = json.loads(subprocess.check_output(['hyprctl', 'clients', '-j'], timeout=3))
            for client in clients:
                if client.get('title') != 'ASV':
                    continue
                try:
                    command = pathlib.Path(f'/proc/{int(client["pid"])}/cmdline').read_bytes().decode().split('\0')
                    index = command.index('--crane-campus-output')
                except (OSError, ValueError, KeyError):
                    continue
                if '--crane-campus' not in command or pathlib.Path(command[index + 1]).resolve() != args.run.resolve():
                    continue
                address = client['address']
                if not re.fullmatch(r'0x[0-9a-fA-F]+', address):
                    raise ValueError('Invalid compositor window address')
                code = ('hl.dispatch(hl.dsp.window.set_prop({prop="render_unfocused", '
                        f'value="true", window="address:{address}"}}))')
                result = subprocess.run(['hyprctl', 'eval', code], capture_output=True, text=True, timeout=3)
                if result.returncode or result.stdout.strip() != 'ok':
                    report['reason'] = 'Compositor does not support the per-window override.'
                    return
                report.update(state='APPLIED', pid=client['pid'], address=address,
                              property='render_unfocused', value=True)
                return
            time.sleep(.25)
        report['reason'] = 'Campus window did not appear within 90 seconds.'
    except (OSError, ValueError, IndexError, subprocess.SubprocessError) as exc:
        report['reason'] = str(exc)
    finally:
        (args.run / 'desktop-render.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
