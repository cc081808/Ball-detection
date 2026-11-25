from contextlib import redirect_stderr, redirect_stdout
from getpass import GetPassWarning, getpass
from os import devnull, environ
from pathlib import Path
from tempfile import TemporaryDirectory
from warnings import catch_warnings, simplefilter

from roboflow import Roboflow

WORKSPACE = "ct-c-fwa4v"
PROJECT = "ping-pong-ball-detection-kjmcq-5fze6"
VERSION = 1


def main():
    api_key = environ.get("ROBOFLOW_API_KEY", "").strip()
    if not api_key:
        try:
            with catch_warnings():
                simplefilter("error", GetPassWarning)
                api_key = getpass("Paste your Roboflow private API key: ").strip()
        except (EOFError, GetPassWarning, KeyboardInterrupt):
            raise SystemExit("Run this script in the VS Code terminal to enter the key privately.") from None
    if not api_key:
        raise SystemExit("A Roboflow private API key is required.")

    output_dir = Path(__file__).resolve().parent / "downloaded_model"
    stage = "authenticate with Roboflow"
    try:
        # SDK diagnostics can contain URLs with credentials.
        with open(devnull, "w", encoding="utf-8") as quiet:
            with redirect_stdout(quiet), redirect_stderr(quiet):
                workspace = Roboflow(api_key=api_key).workspace(WORKSPACE)
            print(f"Authenticated with Roboflow workspace {WORKSPACE}.", flush=True)

            stage = f"look up trained models for {PROJECT} version {VERSION}"
            with redirect_stdout(quiet), redirect_stderr(quiet):
                models = workspace.project(PROJECT).version(VERSION).models()
            if not models:
                raise SystemExit(f"No trained models were found for {PROJECT} version {VERSION}.")
            if len(models) > 1:
                raise SystemExit(f"Found {len(models)} trained models. Model IDs: " +
                                 ", ".join(model.model_id for model in models))

            stage = "download the model weights"
            print(f"Downloading weights for {models[0].model_id}...", flush=True)
            output_dir.mkdir(exist_ok=True)
            with TemporaryDirectory(dir=output_dir) as temporary_dir:
                with redirect_stdout(quiet), redirect_stderr(quiet):
                    downloaded = Path(models[0].download(location=temporary_dir))
                file_size = downloaded.stat().st_size
                if file_size == 0:
                    raise SystemExit("Roboflow returned an empty weights file.")
                destination = output_dir / downloaded.name
                downloaded.replace(destination)

    except Exception as error:
        status = getattr(getattr(error, "response", None), "status_code", None)
        if not isinstance(status, int):
            status = getattr(error, "status_code", None)
        detail = f"HTTP {status}" if isinstance(status, int) else type(error).__name__
        raise SystemExit(f"Could not {stage} ({detail}).") from None

    print(f"Verified download: {destination} ({file_size:,} bytes)")


if __name__ == "__main__":
    main()
