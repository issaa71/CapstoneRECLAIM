"""
manual_teach.py — Interactive arm teaching tool for RECLAIM.

Stand beside the arm and use this to jog joints, save poses,
and test bin sequences. Saves poses to both Teensy RAM and
a local YAML file.

Usage:
    python3 manual_teach.py --port /dev/ttyACM0 --poses-file poses.yaml

Authors: Shady Siam, Issa Ahmed
"""

from __future__ import annotations

import argparse
import shlex
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

from teensy_client import TeensyArmClient


HELP_TEXT = """
Commands:
  help
  quit / exit
  ports

  print
  list
  locals

  arm           Attach servos (send ARM to Teensy)
  disarm        Detach servos (send DISARM to Teensy)

  nudge JOINT DELTA [REPEAT]
    example: nudge 1 1
    example: nudge 2 -0.5 4

  pose NAME [T_MS]
    example: pose home
    example: pose home 1200

  set J1 J2 J3 J4 J5 J6 [T_MS]
    example: set 0 5 10 0 0 0
    example: set 0 5 10 0 0 0 1200

  goto J1 J2 J3 J4 J5 J6
    example: goto 0 0 0 0 0 0

  save NAME [T_MS]
    - gets current state from Teensy
    - sends SAVE NAME to Teensy RAM
    - writes pose into poses.yaml locally

  delete NAME
    - deletes pose from Teensy RAM
    - removes pose from poses.yaml locally

  runbin N
    example: runbin 1

  grip ANGLE [T_MS]
    Move only gripper (J6) to ANGLE
    example: grip 50
    example: grip 50 300

  pickup BIN_N GRIP_ANGLE [GRIP_T_MS]
    Full pick-and-sort cycle
    example: pickup 1 50
    example: pickup 2 45 300

  raw <any Teensy command>
    example: raw HELP
    example: raw POSE home T 1200
"""


def load_pose_db(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {"poses": {}, "sequences": {}}

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if not isinstance(data, dict):
        data = {}

    data.setdefault("poses", {})
    data.setdefault("sequences", {})
    return data


def save_pose_db(path: Path, db: Dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(db, f, sort_keys=False)


def pretty_state(state: Optional[Dict[int, float]]) -> None:
    if not state:
        print("No valid 6-joint state parsed.")
        return

    vals = [state[i] for i in range(1, 7)]
    print("Current q:", " ".join(f"{v:.2f}" for v in vals))


def ensure_pose_db_entry(
    db: Dict[str, Any], name: str, q: List[float], t_ms: int
) -> None:
    db.setdefault("poses", {})
    db["poses"][name] = {
        "q": [float(v) for v in q],
        "t_ms": int(t_ms),
    }


def remove_pose_db_entry(db: Dict[str, Any], name: str) -> None:
    if "poses" in db and name in db["poses"]:
        del db["poses"][name]


def print_local_poses(db: Dict[str, Any]) -> None:
    poses = db.get("poses", {})
    if not poses:
        print("(no local poses)")
        return

    print("Local poses:")
    for name, entry in poses.items():
        q = entry.get("q", [])
        t_ms = entry.get("t_ms", "")
        print(f"  {name}: q={q}, t_ms={t_ms}")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="RECLAIM arm teaching tool")
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
    return p.parse_args()


def main() -> int:
    args = parse_args()
    poses_path = Path(args.poses_file)
    pose_db = load_pose_db(poses_path)

    print("Available ports:")
    for line in TeensyArmClient.list_ports():
        print(" ", line)

    print()
    print(f"Connecting to {args.port} at {args.baud}...")
    print("Make sure Arduino Serial Monitor / pio device monitor is closed.")
    print()

    with TeensyArmClient(args.port, args.baud) as arm:
        print("Connected.")
        print(HELP_TEXT)

        while True:
            try:
                raw_in = input("teach> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not raw_in:
                continue

            parts = shlex.split(raw_in)
            cmd = parts[0].lower()

            try:
                if cmd in {"quit", "exit"}:
                    break

                elif cmd == "help":
                    print(HELP_TEXT)

                elif cmd == "ports":
                    for line in TeensyArmClient.list_ports():
                        print(" ", line)

                elif cmd == "print":
                    state = arm.get_state(echo=True)
                    pretty_state(state)

                elif cmd == "list":
                    arm.command(
                        "LIST",
                        wait_for_done=False,
                        timeout_s=2.0,
                        quiet_s=0.35,
                        echo=True,
                    )

                elif cmd == "locals":
                    print_local_poses(pose_db)

                elif cmd == "arm":
                    arm.command(
                        "ARM",
                        wait_for_done=False,
                        timeout_s=2.0,
                        quiet_s=0.35,
                        echo=True,
                    )

                elif cmd == "disarm":
                    arm.command(
                        "DISARM",
                        wait_for_done=False,
                        timeout_s=2.0,
                        quiet_s=0.35,
                        echo=True,
                    )

                elif cmd == "nudge":
                    if len(parts) not in {3, 4}:
                        print("Usage: nudge JOINT DELTA [REPEAT]")
                        continue

                    joint = int(parts[1])
                    delta = float(parts[2])
                    repeat = int(parts[3]) if len(parts) == 4 else 1

                    for i in range(repeat):
                        result = arm.command(
                            f"NUDGE {joint} {delta}",
                            wait_for_done=True,
                            timeout_s=5.0,
                            echo=True,
                        )
                        if result.timed_out:
                            print("Timed out waiting for DONE.")
                            break
                        time.sleep(0.05)

                    state = arm.get_state(echo=False)
                    pretty_state(state)

                elif cmd == "pose":
                    if len(parts) not in {2, 3}:
                        print("Usage: pose NAME [T_MS]")
                        continue

                    name = parts[1]
                    if len(parts) == 3:
                        t_ms = int(parts[2])
                        teensy_cmd = f"POSE {name} T {t_ms}"
                        timeout_s = max(5.0, t_ms / 1000.0 + 3.0)
                    else:
                        teensy_cmd = f"POSE {name}"
                        timeout_s = 8.0

                    result = arm.command(
                        teensy_cmd,
                        wait_for_done=True,
                        timeout_s=timeout_s,
                        echo=True,
                    )
                    if result.timed_out:
                        print("Timed out waiting for DONE.")

                elif cmd == "set":
                    if len(parts) not in {7, 8}:
                        print("Usage: set J1 J2 J3 J4 J5 J6 [T_MS]")
                        continue

                    q = [float(x) for x in parts[1:7]]
                    if len(parts) == 8:
                        t_ms = int(parts[7])
                    else:
                        t_ms = 900

                    teensy_cmd = (
                        f"SET {' '.join(str(v) for v in q)} T {t_ms}"
                    )
                    timeout_s = max(5.0, t_ms / 1000.0 + 3.0)
                    result = arm.command(
                        teensy_cmd,
                        wait_for_done=True,
                        timeout_s=timeout_s,
                        echo=True,
                    )
                    if result.timed_out:
                        print("Timed out waiting for DONE.")

                elif cmd == "goto":
                    if len(parts) != 7:
                        print("Usage: goto J1 J2 J3 J4 J5 J6")
                        continue

                    q = [float(x) for x in parts[1:7]]
                    teensy_cmd = f"GOTO {' '.join(str(v) for v in q)}"
                    arm.command(
                        teensy_cmd,
                        wait_for_done=False,
                        timeout_s=2.0,
                        quiet_s=0.35,
                        echo=True,
                    )

                elif cmd == "save":
                    if len(parts) not in {2, 3}:
                        print("Usage: save NAME [T_MS]")
                        continue

                    name = parts[1]
                    t_ms = int(parts[2]) if len(parts) == 3 else 1000

                    state = arm.get_state(echo=False)
                    if not state:
                        print(
                            "Could not parse state from Teensy. Save aborted."
                        )
                        continue

                    result = arm.command(
                        f"SAVE {name}",
                        wait_for_done=False,
                        timeout_s=2.0,
                        quiet_s=0.35,
                        echo=True,
                    )
                    q = [state[i] for i in range(1, 7)]
                    ensure_pose_db_entry(pose_db, name, q, t_ms)
                    save_pose_db(poses_path, pose_db)

                    print(
                        f"Saved pose '{name}' to Teensy RAM and {poses_path}."
                    )
                    print("q =", q, "t_ms =", t_ms)

                elif cmd == "delete":
                    if len(parts) != 2:
                        print("Usage: delete NAME")
                        continue

                    name = parts[1]
                    arm.command(
                        f"DELETE {name}",
                        wait_for_done=False,
                        timeout_s=2.0,
                        quiet_s=0.35,
                        echo=True,
                    )
                    remove_pose_db_entry(pose_db, name)
                    save_pose_db(poses_path, pose_db)
                    print(f"Deleted pose '{name}' from local file.")

                elif cmd == "runbin":
                    if len(parts) != 2:
                        print("Usage: runbin N")
                        continue

                    n = int(parts[1])
                    result = arm.command(
                        f"RUNBIN {n}",
                        wait_for_done=True,
                        timeout_s=20.0,
                        echo=True,
                    )
                    if result.timed_out:
                        print("Timed out waiting for RUNBIN to finish.")

                elif cmd == "grip":
                    if len(parts) not in {2, 3}:
                        print("Usage: grip ANGLE [T_MS]")
                        continue

                    angle = float(parts[1])
                    if len(parts) == 3:
                        t_ms = int(parts[2])
                        teensy_cmd = f"GRIP {angle} T {t_ms}"
                        timeout_s = max(5.0, t_ms / 1000.0 + 3.0)
                    else:
                        teensy_cmd = f"GRIP {angle}"
                        timeout_s = 5.0

                    result = arm.command(
                        teensy_cmd,
                        wait_for_done=True,
                        timeout_s=timeout_s,
                        echo=True,
                    )
                    if result.timed_out:
                        print("Timed out waiting for DONE.")

                elif cmd == "pickup":
                    if len(parts) not in {3, 4}:
                        print("Usage: pickup BIN_N GRIP_ANGLE [GRIP_T_MS]")
                        continue

                    n = int(parts[1])
                    angle = float(parts[2])
                    if len(parts) == 4:
                        t_ms = int(parts[3])
                        teensy_cmd = f"PICKUP {n} {angle} T {t_ms}"
                    else:
                        teensy_cmd = f"PICKUP {n} {angle}"

                    result = arm.command(
                        teensy_cmd,
                        wait_for_done=True,
                        timeout_s=60.0,
                        echo=True,
                    )
                    if result.timed_out:
                        print("Timed out waiting for PICKUP to finish.")

                elif cmd == "raw":
                    if len(parts) < 2:
                        print("Usage: raw <any Teensy command>")
                        continue

                    teensy_cmd = raw_in[len(parts[0]) :].strip()
                    wait = arm.looks_like_motion_command(teensy_cmd)
                    timeout_s = 10.0
                    result = arm.command(
                        teensy_cmd,
                        wait_for_done=wait,
                        timeout_s=timeout_s,
                        echo=True,
                    )
                    if result.timed_out:
                        print("Timed out waiting for command completion.")

                else:
                    print("Unknown command.")
                    print(HELP_TEXT)

            except Exception as e:
                print(f"Error: {e}")

    print("Disconnected.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
