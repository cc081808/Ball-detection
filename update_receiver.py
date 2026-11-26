"""Build and install only the receiver in program slot 4. No actions on import."""
import json
import os
from pathlib import Path
import subprocess


def update(root, runner=subprocess.run, ports=None):
    receiver = root / "vex-receiver"
    settings = json.loads((receiver / ".vscode/vex_project_settings.json").read_text())
    if settings["project"]["slot"] != 4 or settings["project"]["name"] != "ball-serial":
        raise RuntimeError("Expected ball-serial in slot 4. No download performed.")
    base = Path(os.environ["APPDATA"]) / "Code/User/globalStorage/vexrobotics.vexcode"
    toolchain = base / "tools/cpp/toolchain_win32"
    env = os.environ.copy()
    env["PATH"] = os.pathsep.join(str(toolchain / path) for path in
                                 ("clang/bin", "gcc/bin", "tools/bin")) + os.pathsep + env["PATH"]
    result = runner([str(toolchain / "tools/bin/make.exe"), "P=ball-serial",
                     f"T={base / 'sdk/cpp/IQ2/IQ2_20250520_15_00_00'}"], cwd=receiver, env=env)
    if result.returncode:
        raise RuntimeError("Build failed. Nothing downloaded; the old binary was not used.")
    binary = receiver / "build/ball-serial.bin"
    built = binary.read_bytes()
    if b"SENSOR + CV V5.1" not in built or b"BALL_SERIAL_V5 " not in built:
        raise RuntimeError("Build does not contain the sensor + CV confirmation update. Nothing downloaded.")
    if ports is None:
        from serial.tools.list_ports import comports
        ports = list(comports())
    matches = [p for p in ports if "vex iq" in p.description.lower() and "communications" in p.description.lower()]
    if len(matches) != 1:
        raise RuntimeError("Connect one IQ Brain directly by USB, then run this updater again.")
    vexcom = base / "tools/vexcom/1_0_2_2/win32/vexcom.exe"
    print(f"Downloading ball-serial to PROGRAM SLOT 4 on {matches[0].device}...", flush=True)
    result = runner([str(vexcom), "--slot", "4", "--write", str(binary), "--run", matches[0].device], cwd=receiver)
    if result.returncode:
        raise RuntimeError("Download failed. Copy the output above; close the VEX serial monitor before retrying.")
    print("Slot 4 updated and started IDLE. Other program slots were not targeted.")
    print("Run 3 - Start VEX link.cmd. Verify SENSOR + CV V5.1 on the Brain; tap LED after camera connects.")


if __name__ == "__main__":
    try:
        update(Path(__file__).resolve().parent)
    except (RuntimeError, OSError, KeyError, ValueError) as error:
        raise SystemExit(str(error)) from None
