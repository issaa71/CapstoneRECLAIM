"""
teensy_client.py — Shared serial backend for RECLAIM arm controller.

Provides TeensyArmClient class for communicating with the Teensy arm
controller firmware over serial. Used by manual_teach.py and pose_runner.py.

Authors: Shady Siam, Issa Ahmed
"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import serial
import serial.tools.list_ports


STATE_RE = re.compile(r"J(\d+):\s*q=([-+]?\d*\.?\d+)")


@dataclass
class CommandResult:
    command: str
    lines: List[str]
    done_received: bool = False
    timed_out: bool = False


class TeensyArmClient:
    def __init__(
        self,
        port: str,
        baud: int = 115200,
        timeout_s: float = 0.05,
        boot_wait_s: float = 2.0,
    ) -> None:
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self.boot_wait_s = boot_wait_s
        self.ser: Optional[serial.Serial] = None

    def open(self) -> None:
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout_s)
        time.sleep(self.boot_wait_s)
        self.flush_input()

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __enter__(self) -> "TeensyArmClient":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @staticmethod
    def list_ports() -> List[str]:
        ports = []
        for p in serial.tools.list_ports.comports():
            desc = p.description or ""
            ports.append(f"{p.device}  |  {desc}")
        return ports

    @staticmethod
    def looks_like_motion_command(cmd: str) -> bool:
        c = cmd.strip().upper()
        return (
            c.startswith("SET ")
            or c.startswith("POSE ")
            or c.startswith("RUNBIN")
            or c.startswith("NUDGE ")
            or c.startswith("GRIP")
            or c.startswith("PICKUP")
        )

    def flush_input(self) -> None:
        if not self.ser:
            return
        self.ser.reset_input_buffer()

    def send_line(self, cmd: str) -> None:
        if not self.ser:
            raise RuntimeError("Serial port is not open.")
        self.ser.write((cmd.strip() + "\n").encode("utf-8"))

    def _readline(self) -> Optional[str]:
        if not self.ser:
            raise RuntimeError("Serial port is not open.")
        raw = self.ser.readline()
        if not raw:
            return None
        return raw.decode("utf-8", errors="ignore").strip()

    def command(
        self,
        cmd: str,
        wait_for_done: bool = False,
        timeout_s: float = 10.0,
        quiet_s: float = 0.25,
        echo: bool = True,
    ) -> CommandResult:
        if not self.ser:
            raise RuntimeError("Serial port is not open.")

        self.flush_input()
        if echo:
            print(f">> {cmd}")
        self.send_line(cmd)

        lines: List[str] = []
        start = time.time()
        last_rx = time.time()
        done_received = False

        while True:
            line = self._readline()
            if line is not None and line != "":
                lines.append(line)
                last_rx = time.time()
                if echo:
                    print(line)
                if line == "DONE":
                    done_received = True
                    if wait_for_done:
                        return CommandResult(
                            cmd, lines, done_received=True, timed_out=False
                        )
            else:
                now = time.time()

                if wait_for_done:
                    if now - start > timeout_s:
                        return CommandResult(
                            cmd, lines, done_received=False, timed_out=True
                        )
                else:
                    if (now - last_rx) > quiet_s:
                        return CommandResult(
                            cmd,
                            lines,
                            done_received=done_received,
                            timed_out=False,
                        )

            time.sleep(0.01)

    def get_state(self, echo: bool = True) -> Optional[Dict[int, float]]:
        result = self.command(
            "PRINT", wait_for_done=False, timeout_s=2.0, quiet_s=0.35, echo=echo
        )
        state = self.parse_state_lines(result.lines)
        return state

    @staticmethod
    def parse_state_lines(lines: List[str]) -> Optional[Dict[int, float]]:
        q: Dict[int, float] = {}
        for line in lines:
            m = STATE_RE.search(line)
            if m:
                joint_idx = int(m.group(1))
                joint_q = float(m.group(2))
                q[joint_idx] = joint_q

        if len(q) == 6:
            return q
        return None
