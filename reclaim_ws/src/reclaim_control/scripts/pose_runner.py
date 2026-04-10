"""
pose_runner.py — Run poses and sequences from YAML for RECLAIM arm.

Supports two modes:
  - set mode: sends SET ... T ... using YAML angles (MIC-like workflow)
  - pose mode: sends POSE name T ms using Teensy's RAM pose table

Usage:
    python3 pose_runner.py --port /dev/ttyACM0 list
    python3 pose_runner.py --port /dev/ttyACM0 --mode set pose home
    python3 pose_runner.py --port /dev/ttyACM0 --mode set seq bin1_cycle

Authors: Shady Siam, Issa Ahmed
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict

import yaml

from teensy_client import TeensyArmClient


def load_pose_db(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"Pose file not found: {path}")

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if not isinstance(data, dict):
        raise ValueError("Pose file is not a YAML dictionary.")

    data.setdefault("poses", {})
    data.setdefault("sequences", {})
    return data


def build_set_command(
    entry: Dict[str, Any], override_t_ms: int | None = None
) -> str:
    q = entry.get("q")
    if not isinstance(q, list) or len(q) != 6:
        raise ValueError("Each pose must have q: [j1, j2, j3, j4, j5, j6]")

    t_ms = int(
        override_t_ms if override_t_ms is not None else entry.get("t_ms", 900)
    )
    return f"SET {' '.join(str(float(v)) for v in q)} T {t_ms}"


def build_pose_command(
    name: str, entry: Dict[str, Any], override_t_ms: int | None = None
) -> str:
    t_ms = int(
        override_t_ms if override_t_ms is not None else entry.get("t_ms", 900)
    )
    return f"POSE {name} T {t_ms}"


def print_db_summary(db: Dict[str, Any]) -> None:
    poses = db.get("poses", {})
    seqs = db.get("sequences", {})

    print("Poses:")
    if poses:
        for name, entry in poses.items():
            print(f"  {name}: q={entry.get('q')}, t_ms={entry.get('t_ms')}")
    else:
        print("  (none)")

    print("\nSequences:")
    if seqs:
        for name, steps in seqs.items():
            print(f"  {name}: {len(steps)} steps")
    else:
        print("  (none)")


def run_one_pose(
    arm: TeensyArmClient,
    db: Dict[str, Any],
    pose_name: str,
    mode: str,
    override_t_ms: int | None = None,
) -> None:
    poses = db.get("poses", {})
    if pose_name not in poses:
        raise KeyError(f"Pose '{pose_name}' not found in YAML.")

    entry = poses[pose_name]

    if mode == "set":
        cmd = build_set_command(entry, override_t_ms)
        t_ms = int(
            override_t_ms
            if override_t_ms is not None
            else entry.get("t_ms", 900)
        )
    elif mode == "pose":
        cmd = build_pose_command(pose_name, entry, override_t_ms)
        t_ms = int(
            override_t_ms
            if override_t_ms is not None
            else entry.get("t_ms", 900)
        )
    else:
        raise ValueError("Mode must be 'set' or 'pose'.")

    timeout_s = max(5.0, t_ms / 1000.0 + 3.0)
    result = arm.command(cmd, wait_for_done=True, timeout_s=timeout_s, echo=True)

    if result.timed_out:
        raise TimeoutError(
            f"Timed out waiting for pose '{pose_name}' to finish."
        )


def run_sequence(
    arm: TeensyArmClient,
    db: Dict[str, Any],
    seq_name: str,
    mode: str,
    grip_angle: float | None = None,
) -> None:
    seqs = db.get("sequences", {})
    if seq_name not in seqs:
        raise KeyError(f"Sequence '{seq_name}' not found in YAML.")

    steps = seqs[seq_name]
    if not isinstance(steps, list):
        raise ValueError("Sequence must be a list.")

    for idx, step in enumerate(steps, start=1):
        if isinstance(step, str):
            pose_name = step
            override_t_ms = None
            print(f"\n--- Step {idx}/{len(steps)}: {pose_name} ---")
            run_one_pose(arm, db, pose_name, mode, override_t_ms)

        elif isinstance(step, dict):
            if "action" in step and step["action"] == "grip":
                angle = (
                    grip_angle
                    if grip_angle is not None
                    else step.get("angle", 50)
                )
                t_ms = step.get("t_ms", 500)
                print(f"\n--- Step {idx}/{len(steps)}: grip {angle} ---")
                timeout_s = max(5.0, t_ms / 1000.0 + 3.0)
                result = arm.command(
                    f"GRIP {angle} T {t_ms}",
                    wait_for_done=True,
                    timeout_s=timeout_s,
                    echo=True,
                )
                if result.timed_out:
                    raise TimeoutError("Timed out on grip step.")
            else:
                pose_name = step.get("pose")
                override_t_ms = step.get("t_ms")
                if pose_name is None:
                    raise ValueError(
                        f"Sequence step {idx} is missing 'pose' or 'action'."
                    )
                print(f"\n--- Step {idx}/{len(steps)}: {pose_name} ---")
                run_one_pose(arm, db, pose_name, mode, override_t_ms)

        else:
            raise ValueError(f"Invalid sequence step at index {idx}.")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="RECLAIM arm pose runner")
    p.add_argument(
        "--port",
        required=True,
        help="Serial port, e.g. /dev/ttyACM0 or COM5",
    )
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument(
        "--poses-file",
        default=str(
            Path(__file__).resolve().parent.parent / "config" / "poses.yaml"
        ),
        help="Path to poses YAML file (default: ../config/poses.yaml)",
    )
    p.add_argument("--mode", choices=["set", "pose"], default="set")

    sub = p.add_subparsers(dest="action", required=True)

    sub.add_parser("list", help="List poses and sequences from YAML")

    p_pose = sub.add_parser("pose", help="Run a single named pose")
    p_pose.add_argument("name")
    p_pose.add_argument("--t-ms", type=int, default=None)

    p_seq = sub.add_parser("seq", help="Run a named sequence")
    p_seq.add_argument("name")
    p_seq.add_argument(
        "--grip-angle",
        type=float,
        default=None,
        help="Override grip angle for pickup sequences",
    )

    return p.parse_args()


def main() -> int:
    args = parse_args()
    db = load_pose_db(Path(args.poses_file))

    if args.action == "list":
        print_db_summary(db)
        return 0

    print("Available ports:")
    for line in TeensyArmClient.list_ports():
        print(" ", line)

    print()
    print(f"Connecting to {args.port} at {args.baud}...")
    print("Make sure Arduino Serial Monitor / pio device monitor is closed.")
    print()

    with TeensyArmClient(args.port, args.baud) as arm:
        # Auto-arm before running poses
        print("Sending ARM...")
        arm.command(
            "ARM",
            wait_for_done=False,
            timeout_s=2.0,
            quiet_s=0.35,
            echo=True,
        )

        if args.action == "pose":
            run_one_pose(arm, db, args.name, args.mode, args.t_ms)
        elif args.action == "seq":
            grip_angle = getattr(args, "grip_angle", None)
            run_sequence(arm, db, args.name, args.mode, grip_angle)

    print("\nDone.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
