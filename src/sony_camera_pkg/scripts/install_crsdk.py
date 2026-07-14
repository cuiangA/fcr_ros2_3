#!/usr/bin/env python3
"""Stage a user-downloaded Sony Camera Remote SDK for this ROS 2 package."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


REQUIRED_LIBRARIES = (
    "libCr_Core.so",
    "libmonitor_protocol.so",
    "libmonitor_protocol_pf.so",
    "CrAdapter/libCr_PTP_IP.so",
    "CrAdapter/libCr_PTP_USB.so",
    "CrAdapter/libssh2.so",
    "CrAdapter/libusb-1.0.so",
)


def parse_args() -> argparse.Namespace:
    package_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description=(
            "Copy CRSDK public headers and ARM64 runtime libraries from the "
            "extracted Sony RemoteCli directory into the ignored local sdk directory."
        )
    )
    parser.add_argument(
        "remote_cli_root",
        type=Path,
        help="Path containing app/CRSDK and external/crsdk",
    )
    parser.add_argument(
        "--destination",
        type=Path,
        default=package_root / "sdk",
        help="Staging root; defaults to sony_camera_pkg/sdk",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = args.remote_cli_root.expanduser().resolve()
    destination = args.destination.expanduser().resolve()
    headers = source / "app" / "CRSDK"
    libraries = source / "external" / "crsdk"

    missing = [
        str(path)
        for path in (headers / "CameraRemote_SDK.h", libraries / "libCr_Core.so")
        if not path.is_file()
    ]
    missing.extend(
        str(libraries / relative)
        for relative in REQUIRED_LIBRARIES
        if not (libraries / relative).is_file()
    )
    if missing:
        raise SystemExit("CRSDK source is incomplete; missing:\n  " + "\n  ".join(missing))

    include_destination = destination / "include" / "CRSDK"
    library_destination = destination / "lib"
    include_destination.mkdir(parents=True, exist_ok=True)
    library_destination.mkdir(parents=True, exist_ok=True)
    shutil.copytree(headers, include_destination, dirs_exist_ok=True)
    shutil.copytree(libraries, library_destination, dirs_exist_ok=True)

    print(f"CRSDK headers staged at: {include_destination}")
    print(f"CRSDK libraries staged at: {library_destination}")
    print("Build with -DSONY_CRSDK_REQUIRED=ON on the Jetson deployment workspace.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
