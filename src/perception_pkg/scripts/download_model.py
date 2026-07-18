#!/usr/bin/env python3
"""Download and verify the frozen perception model from a GitHub Release."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
from urllib.request import Request, urlopen


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    source_manifest = script_dir.parent / "models" / "manifest.json"
    installed_manifest = script_dir.parents[1] / "share" / "perception_pkg" / "models" / "manifest.json"
    default_manifest = source_manifest if source_manifest.exists() else installed_manifest
    parser = argparse.ArgumentParser(
        description="Download yolov8n.onnx and reject incomplete or modified files."
    )
    parser.add_argument("--manifest", type=Path, default=default_manifest)
    parser.add_argument("--output", type=Path, help="Destination file or directory")
    parser.add_argument("--url", help="Override the Release URL in the manifest")
    parser.add_argument("--force", action="store_true", help="Replace a valid existing file")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    manifest_path = args.manifest.resolve()
    with manifest_path.open("r", encoding="utf-8") as stream:
        manifest = json.load(stream)

    filename = str(manifest["filename"])
    expected = str(manifest["sha256"]).lower()
    url = args.url or str(manifest["url"])
    if args.output is None:
        destination = manifest_path.parent / filename
    else:
        requested = args.output.expanduser().resolve()
        destination = (
            requested / filename
            if requested.is_dir() or requested.suffix == ""
            else requested
        )

    if destination.exists() and not args.force:
        actual = sha256(destination)
        if actual == expected:
            print(f"Model already verified: {destination}")
            return 0
        print(
            f"Refusing to overwrite checksum-mismatched file: {destination}\n"
            "Use --force only if replacing it is intentional.",
            file=sys.stderr,
        )
        return 2

    destination.parent.mkdir(parents=True, exist_ok=True)
    request = Request(url, headers={"User-Agent": "fcr_ros2-model-downloader/1"})
    temporary_path: Path | None = None
    try:
        with urlopen(request, timeout=60) as response, tempfile.NamedTemporaryFile(
            prefix=f".{filename}.", suffix=".part", dir=destination.parent, delete=False
        ) as temporary:
            temporary_path = Path(temporary.name)
            shutil.copyfileobj(response, temporary)

        actual = sha256(temporary_path)
        if actual != expected:
            raise RuntimeError(
                f"SHA-256 mismatch: expected {expected}, downloaded {actual}"
            )
        os.replace(temporary_path, destination)
        temporary_path = None
        print(f"Downloaded and verified: {destination}")
        return 0
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"Model download failed: {error}", file=sys.stderr)
        raise SystemExit(1)
