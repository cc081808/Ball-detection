"""Build and install only cv-collector in slot 5; refuse unrelated programs."""
import argparse
import json
import os
from pathlib import Path
import subprocess


def programs_from_json(text):
    try:
        start = text.index('{')
        data, _ = json.JSONDecoder().raw_decode(text[start:])
    except (ValueError, TypeError) as error:
        raise RuntimeError('Could not read the Brain slot list. Nothing downloaded.') from error
    found = []
    def visit(obj):
        if isinstance(obj, dict):
            if 'programs' in obj:
                found.append(obj['programs'])
            for value in obj.values():
                visit(value)
        elif isinstance(obj, list):
            for value in obj:
                visit(value)
    visit(data)
    if len(found) != 1 or not isinstance(found[0], dict) or not isinstance(found[0].get('items'), list):
        raise RuntimeError('Unrecognized Brain slot list. Nothing downloaded.')
    result = {}
    for entry in found[0]['items']:
        if entry is None:
            continue
        slot = entry.get('slot')
        if type(slot) is not int or not 0 <= slot <= 7 or not isinstance(entry.get('file'), str) or slot in result:
            raise RuntimeError('Invalid Brain slot list. Nothing downloaded.')
        result[slot] = (entry['file'], entry.get('size'), entry.get('type'))
    return result


def update(root, runner=subprocess.run, ports=None, build_only=False):
    project = root / 'vex-cv'
    settings = json.loads((project / '.vscode/vex_project_settings.json').read_text())['project']
    if settings['slot'] != 5 or settings['name'] != 'cv-collector' or settings['platform'] != 'IQ2':
        raise RuntimeError('Expected cv-collector in IQ2 slot 5. Nothing downloaded.')
    base = Path(os.environ['APPDATA']) / 'Code/User/globalStorage/vexrobotics.vexcode'
    toolchain = base / 'tools/cpp/toolchain_win32'
    env = os.environ.copy()
    env['PATH'] = os.pathsep.join(str(toolchain / p) for p in ('clang/bin','gcc/bin','tools/bin')) + os.pathsep + env['PATH']
    result = runner([str(toolchain / 'tools/bin/make.exe'), '-B', 'P=cv-collector',
                     f"T={base / 'sdk/cpp/IQ2/IQ2_20250520_15_00_00'}"], cwd=project, env=env)
    if result.returncode:
        raise RuntimeError('Build failed. Nothing downloaded; an older binary will not be used.')
    binary = project / 'build/cv-collector.bin'
    contents = binary.read_bytes()
    if not 100 < len(contents) <= 98304 or b'CV SLOT 5 V1' not in contents or b'CV_COORD_V1 DRIVE' not in contents:
        raise RuntimeError('Wrong or incomplete slot 5 binary. Nothing downloaded.')
    if build_only:
        print('Slot 5 build passed. No Brain connection or download requested.')
        return
    if ports is None:
        from serial.tools.list_ports import comports
        ports = list(comports())
    matches = [p for p in ports if 'vex iq' in p.description.lower() and 'communications' in p.description.lower()]
    if len(matches) != 1:
        raise RuntimeError('Build complete. Connect one powered IQ Brain by USB and run this installer again.')
    port = matches[0].device
    vexcom = str(base / 'tools/vexcom/1_0_2_2/win32/vexcom.exe')
    def slots():
        result = runner([vexcom, '--json', port], capture_output=True, text=True, timeout=15)
        if result.returncode:
            raise RuntimeError('Could not read the Brain. Close the VEX serial monitor and retry.')
        return programs_from_json(result.stdout)
    before = slots()
    if 4 in before and before[4][0] != 'cv-collector':
        raise RuntimeError(f'Program slot 5 contains {before[4][0]!r}. It was not overwritten.')
    print('Installing cv-collector in PROGRAM SLOT 5; starts idle after calibration.', flush=True)
    result = runner([vexcom, '--slot', '5', '--write', str(binary), '--run', port], cwd=project)
    if result.returncode:
        raise RuntimeError('Slot 5 download failed. Close the VEX serial monitor and retry.')
    after = slots()
    if after.get(4, ('',))[0] != 'cv-collector':
        raise RuntimeError('Download returned, but slot 5 verification failed. Check the Brain Programs menu.')
    if {k:v for k,v in before.items() if k != 4} != {k:v for k,v in after.items() if k != 4}:
        raise RuntimeError('Other slot listings changed unexpectedly. Inspect the Brain before running.')
    print('Verified slot 5. Other program slots unchanged. Run 7 - Start CV slot 5.cmd.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-only', action='store_true')
    args = parser.parse_args()
    try:
        update(Path(__file__).resolve().parent, build_only=args.build_only)
    except (RuntimeError, OSError, KeyError, ValueError, subprocess.TimeoutExpired) as error:
        raise SystemExit(str(error)) from None
