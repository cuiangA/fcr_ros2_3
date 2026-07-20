#!/usr/bin/env python3
"""Safe, base-only LeKiwi hardware acceptance test for LeRobot v0.6.0.

This script never changes motor IDs and never touches arm motors 1..6.
Motion is opt-in, low speed, short duration, and always followed by repeated zero commands.
"""

from __future__ import annotations

import argparse
import json
import math
import platform
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode


MOTOR_NAMES = ("base_left_wheel", "base_back_wheel", "base_right_wheel")
MOTOR_IDS = (7, 8, 9)
WHEEL_ANGLES_RAD = tuple(math.radians(v) for v in (150.0, -90.0, 30.0))
WHEEL_RADIUS_M = 0.05
BASE_RADIUS_M = 0.125
STEPS_PER_RAD = 4096.0 / (2.0 * math.pi)


def body_to_raw(vx: float, vy: float, wz: float, max_raw: int) -> dict[str, int]:
    values = []
    for angle in WHEEL_ANGLES_RAD:
        wheel_rad_s = (
            math.cos(angle) * vx
            + math.sin(angle) * vy
            + BASE_RADIUS_M * wz
        ) / WHEEL_RADIUS_M
        values.append(int(round(wheel_rad_s * STEPS_PER_RAD)))
    peak = max(abs(v) for v in values)
    if peak > max_raw:
        scale = max_raw / peak
        values = [int(round(v * scale)) for v in values]
    return dict(zip(MOTOR_NAMES, values, strict=True))


def stop_base(bus: FeetechMotorsBus) -> None:
    zeros = dict.fromkeys(MOTOR_NAMES, 0)
    for _ in range(3):
        try:
            bus.sync_write("Goal_Velocity", zeros)
        except Exception as exc:  # Keep trying the remaining safety writes.
            print(f"WARNING: zero-velocity write failed: {exc}", file=sys.stderr)
        time.sleep(0.05)


def make_bus(port: str) -> FeetechMotorsBus:
    motors = {
        name: Motor(motor_id, "sts3215", MotorNormMode.RANGE_M100_100)
        for name, motor_id in zip(MOTOR_NAMES, MOTOR_IDS, strict=True)
    }
    return FeetechMotorsBus(port=port, motors=motors)


def configure_motion(bus: FeetechMotorsBus) -> None:
    bus.disable_torque(list(MOTOR_NAMES))
    bus.configure_motors()
    for name in MOTOR_NAMES:
        bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
    bus.enable_torque(list(MOTOR_NAMES))
    stop_base(bus)


def read_state(bus: FeetechMotorsBus) -> dict[str, object]:
    velocity = bus.sync_read("Present_Velocity", list(MOTOR_NAMES))
    voltage = {name: bus.read("Present_Voltage", name) for name in MOTOR_NAMES}
    return {"velocity_raw": velocity, "voltage_v": voltage}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Prefer /dev/serial/by-id/... over /dev/ttyACM0")
    parser.add_argument("--motion", action="store_true", help="Enable six short wheel-off-ground motion tests")
    parser.add_argument(
        "--confirm-wheels-off-ground",
        action="store_true",
        help="Required with --motion; confirms all three wheels are suspended",
    )
    parser.add_argument("--duration", type=float, default=0.4, help="Seconds per motion, maximum 0.5")
    parser.add_argument("--max-raw", type=int, default=350, help="Raw wheel speed ceiling, maximum 500")
    parser.add_argument("--report", type=Path, default=Path("lekiwi_phase1_report.json"))
    args = parser.parse_args()
    if args.motion and not args.confirm_wheels_off_ground:
        parser.error("--motion requires --confirm-wheels-off-ground")
    if not 0.1 <= args.duration <= 0.5:
        parser.error("--duration must be between 0.1 and 0.5 seconds")
    if not 50 <= args.max_raw <= 500:
        parser.error("--max-raw must be between 50 and 500")
    return args


def main() -> int:
    args = parse_args()
    report: dict[str, object] = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "port": args.port,
        "platform": platform.platform(),
        "python": sys.version,
        "motor_ids": dict(zip(MOTOR_NAMES, MOTOR_IDS, strict=True)),
        "probe": None,
        "motions": [],
        "result": "FAILED",
    }
    bus = make_bus(args.port)
    try:
        print(f"Connecting to {args.port}; expecting STS3215 IDs 7, 8, 9...")
        bus.connect()
        report["probe"] = read_state(bus)
        print(json.dumps(report["probe"], indent=2, ensure_ascii=False))
        print("PROBE PASS: all three base motors responded.")

        if args.motion:
            configure_motion(bus)
            tests = (
                ("forward", 0.02, 0.0, 0.0),
                ("backward", -0.02, 0.0, 0.0),
                ("left", 0.0, 0.02, 0.0),
                ("right", 0.0, -0.02, 0.0),
                ("rotate_ccw", 0.0, 0.0, 0.15),
                ("rotate_cw", 0.0, 0.0, -0.15),
            )
            print("\nMotion test armed. Press ENTER for each short pulse; Ctrl+C stops immediately.")
            for name, vx, vy, wz in tests:
                command = body_to_raw(vx, vy, wz, args.max_raw)
                input(f"[{name}] expected raw {command}. Press ENTER to pulse, or Ctrl+C to abort: ")
                bus.sync_write("Goal_Velocity", command)
                time.sleep(args.duration)
                stop_base(bus)
                state = read_state(bus)
                observed = input("Observed chassis direction (or notes): ").strip()
                report["motions"].append(
                    {"name": name, "command_raw": command, "post_stop": state, "observed": observed}
                )

        report["result"] = "PASS"
        return 0
    except KeyboardInterrupt:
        report["result"] = "ABORTED"
        print("\nABORTED: sending zero velocity.", file=sys.stderr)
        return 130
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        print(f"FAIL: {report['error']}", file=sys.stderr)
        return 1
    finally:
        if bus.is_connected:
            stop_base(bus)
            bus.disconnect(disable_torque=True)
        args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"Report written to {args.report}")


if __name__ == "__main__":
    raise SystemExit(main())
