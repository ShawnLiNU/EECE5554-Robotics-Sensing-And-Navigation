#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Serial Sensor Emulator for EECE 5554 — Robotics Sensing and Navigation

Creates a pseudo-serial port (via Linux pseudoterminals) and streams sensor
data from a recorded text file, emulating a GPS puck, RTK receiver, or
VectorNav IMU. Useful for developing and testing ROS driver nodes without
physical hardware.

Original authors: Jagatpreet Singh Nir, Kris Dorsey, Arun Anbu
Rewritten by: Xian Li
"""

import os
import sys
import pty
import signal
import time
import re
import argparse
from collections import OrderedDict

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────
VALID_DEVICES = ("gps", "rtk", "imu")

# Default output rates (Hz) — how many navigation-sentence groups per second
DEFAULT_RATES = {
    "gps": 1,    # Standard NMEA GPS puck: 1 fix/second
    "rtk": 5,    # RTK receiver: typically 5 Hz
    "imu": 200,  # VectorNav default register 07 value
}

# NMEA sentence types that carry a UTC timestamp in field 1
_TIMESTAMPED = {"$GPGGA", "$GPRMC", "$GNRMC", "$GNGGA"}


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────
def _extract_nmea_time(line: str):
    """Return the UTC time token from an NMEA sentence, or None."""
    if not line.startswith("$"):
        return None
    parts = line.split(",")
    if len(parts) < 2:
        return None
    tag = parts[0]
    if tag in _TIMESTAMPED:
        return parts[1]          # e.g. "194851.000"
    return None


def _strip_terminal_escapes(line: str) -> str:
    """Remove ANSI / VT100 escape sequences that may have been captured
    by minicom or screen during the original recording."""
    return re.sub(r'\x1b\[[^A-Za-z]*[A-Za-z]', '', line)


def group_nmea_by_epoch(lines: list[str]) -> list[list[str]]:
    """Group consecutive NMEA sentences that share the same UTC timestamp
    into a single 'epoch'.  Sentences without a timestamp (e.g. $GPGSA,
    $GPGSV) are attached to the preceding timestamped sentence.

    Returns a list of groups, where each group is a list of raw strings.
    For IMU data (single-line records) each line becomes its own group.
    """
    groups: list[list[str]] = []
    current_group: list[str] = []
    current_time = None

    for raw in lines:
        line = _strip_terminal_escapes(raw).strip()
        if not line or not line.startswith("$"):
            continue

        t = _extract_nmea_time(line)

        if t is not None and t != current_time:
            # New epoch — flush previous group
            if current_group:
                groups.append(current_group)
            current_group = [line]
            current_time = t
        else:
            current_group.append(line)

    if current_group:
        groups.append(current_group)

    return groups


def parse_vn_sample_rate(vn_cmd: str) -> float:
    """Parse a VectorNav write-register command and return the requested
    sample rate in Hz.

    Expected format:  $VNWRG,07,<rate>*XX
    Returns the rate, or raises ValueError on bad input.
    """
    cmd = vn_cmd.strip()
    # Allow with or without leading 'b' (artifact of Python bytes repr)
    if cmd.startswith("b'") or cmd.startswith('b"'):
        cmd = cmd[2:-1]

    parts = cmd.split(",")
    if len(parts) < 3 or parts[0] != "$VNWRG" or parts[1].strip() != "07":
        raise ValueError(
            f"Invalid VectorNav register command: '{vn_cmd}'\n"
            f"Expected format: $VNWRG,07,<rate>*XX"
        )
    rate_str = parts[2].split("*")[0].strip()
    rate = float(rate_str)
    if rate <= 0:
        raise ValueError(f"Sample rate must be positive, got {rate}")
    return rate


# ──────────────────────────────────────────────────────────────────────────────
# Emulator
# ──────────────────────────────────────────────────────────────────────────────
class SerialEmulator:
    """Streams a recorded sensor data file to a Linux pseudo-serial port."""

    def __init__(self, filepath: str, device_type: str, rate_hz: float,
                 loop: bool = True):
        self.filepath = filepath
        self.device_type = device_type
        self.rate_hz = rate_hz
        self.loop = loop
        self._driver_fd = None  # master side of PTY (we write here)
        self._driven_fd = None  # slave side (user reads here)
        self._running = False

    # ── lifecycle ──────────────────────────────────────────────────────────

    def start(self):
        """Open the pseudo-terminal and begin streaming."""
        self._driver_fd, self._driven_fd = pty.openpty()
        slave_name = os.ttyname(self._driven_fd)
        self._running = True

        print(f"\n  Device  : {self.device_type.upper()}")
        print(f"  File    : {self.filepath}")
        print(f"  Rate    : {self.rate_hz} Hz")
        print(f"  Loop    : {'yes' if self.loop else 'no'}")
        print(f"  Port    : {slave_name}")
        print(f"\n  Connect with:  cat {slave_name}")
        print(f"  Stop with  :  Ctrl-C\n")

        try:
            self._stream()
        except KeyboardInterrupt:
            print("\n  Interrupted by user.")
        finally:
            self.stop()

    def stop(self):
        """Close the pseudo-terminal cleanly."""
        self._running = False
        for fd in (self._driver_fd, self._driven_fd):
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
        self._driver_fd = None
        self._driven_fd = None
        print("  Emulator stopped.\n")

    # ── streaming logic ───────────────────────────────────────────────────

    def _load_lines(self) -> list[str]:
        with open(self.filepath, "r") as f:
            return f.readlines()

    def _write_line(self, line: str):
        """Write a single NMEA/VN sentence to the PTY (with CR+LF)."""
        data = (line.rstrip() + "\r\n").encode("utf-8")
        try:
            os.write(self._driver_fd, data)
        except OSError:
            # Reader may have closed — stop gracefully
            self._running = False

    def _stream(self):
        raw_lines = self._load_lines()
        if not raw_lines:
            print("  Warning: data file is empty.")
            return

        if self.device_type == "imu":
            self._stream_imu(raw_lines)
        else:
            self._stream_nmea(raw_lines)

    def _stream_nmea(self, raw_lines: list[str]):
        """Stream GPS/RTK data grouped by timestamp epoch."""
        groups = group_nmea_by_epoch(raw_lines)
        total_epochs = len(groups)
        if total_epochs == 0:
            print("  Warning: no valid NMEA sentences found in file.")
            return

        interval = 1.0 / self.rate_hz
        pass_num = 0

        while self._running:
            pass_num += 1
            t_start = time.monotonic()

            for i, group in enumerate(groups):
                if not self._running:
                    return

                for sentence in group:
                    self._write_line(sentence)

                # Progress every 10 epochs
                if (i + 1) % 10 == 0 or (i + 1) == total_epochs:
                    elapsed = time.monotonic() - t_start
                    sys.stdout.write(
                        f"\r  Pass {pass_num}: epoch {i+1}/{total_epochs}"
                        f"  ({elapsed:.1f}s elapsed)"
                    )
                    sys.stdout.flush()

                # Sleep until next epoch (but not after the last one in
                # non-loop mode)
                if i < total_epochs - 1 or self.loop:
                    time.sleep(interval)

            print()  # newline after progress

            if not self.loop:
                print("  Reached end of file.")
                return

            print("  Restarting from beginning...")

    def _stream_imu(self, raw_lines: list[str]):
        """Stream IMU data line-by-line at the configured rate."""
        clean = []
        for raw in raw_lines:
            line = _strip_terminal_escapes(raw).strip()
            if line.startswith("$"):
                clean.append(line)

        total = len(clean)
        if total == 0:
            print("  Warning: no valid VN sentences found in file.")
            return

        interval = 1.0 / self.rate_hz
        pass_num = 0

        while self._running:
            pass_num += 1
            t_start = time.monotonic()

            for i, line in enumerate(clean):
                if not self._running:
                    return

                self._write_line(line)

                if (i + 1) % 200 == 0 or (i + 1) == total:
                    elapsed = time.monotonic() - t_start
                    sys.stdout.write(
                        f"\r  Pass {pass_num}: line {i+1}/{total}"
                        f"  ({elapsed:.1f}s elapsed)"
                    )
                    sys.stdout.flush()

                if i < total - 1 or self.loop:
                    time.sleep(interval)

            print()

            if not self.loop:
                print("  Reached end of file.")
                return

            print("  Restarting from beginning...")


# ──────────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────────
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Serial sensor emulator for EECE 5554.  Streams GPS, "
                    "RTK, or IMU data to a pseudo-serial port.  Press Ctrl-C "
                    "to stop.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  # GPS (1 Hz, loop)\n"
            "  python3 serial_emulator.py -f GPS_Chicago.txt -d gps\n\n"
            "  # GPS (no loop)\n"
            "  python3 serial_emulator.py -f GPS_Chicago.txt -d gps -l no\n\n"
            "  # RTK (5 Hz)\n"
            "  python3 serial_emulator.py -f rtk_data.txt -d rtk\n\n"
            "  # IMU at default 200 Hz\n"
            "  python3 serial_emulator.py -f imu_data.txt -d imu\n\n"
            "  # IMU — change sampling rate to 40 Hz via VN register write\n"
            '  python3 serial_emulator.py -f imu_data.txt -d imu '
            '-V "$VNWRG,07,40*XX"\n'
        ),
    )

    p.add_argument(
        "-f", "--file", required=True, dest="file",
        help="Path to the recorded data file.",
    )
    p.add_argument(
        "-d", "--device_type", required=True, dest="device_type",
        choices=VALID_DEVICES,
        help="Sensor type: gps, rtk, or imu.",
    )
    p.add_argument(
        "-l", "--loop", default="yes", dest="loop",
        choices=("yes", "no"),
        help="Loop through the file (default: yes).",
    )
    p.add_argument(
        "-V", "--VN_reg", default=None, dest="vn_reg",
        help=(
            "VectorNav write-register command to set IMU sample rate.  "
            'Format: "$VNWRG,07,<rate>*XX".  Only used when device_type '
            "is imu."
        ),
    )

    return p


def main():
    parser = build_parser()
    args = parser.parse_args()

    # ── validate file ─────────────────────────────────────────────────────
    if not os.path.isfile(args.file):
        parser.error(f"File not found: {args.file}")

    # ── determine sample rate ─────────────────────────────────────────────
    rate = DEFAULT_RATES[args.device_type]

    if args.device_type == "imu" and args.vn_reg is not None:
        try:
            rate = parse_vn_sample_rate(args.vn_reg)
            print(f"  VN register command accepted — rate set to {rate} Hz")
        except ValueError as e:
            parser.error(str(e))

    if args.device_type != "imu" and args.vn_reg is not None:
        print("  Warning: -V/--VN_reg is ignored for non-IMU devices.\n")

    # ── run ───────────────────────────────────────────────────────────────
    loop = args.loop == "yes"
    emulator = SerialEmulator(args.file, args.device_type, rate, loop)

    # Handle SIGINT cleanly (redundant with KeyboardInterrupt, but covers
    # edge cases in threaded contexts)
    signal.signal(signal.SIGINT, lambda *_: emulator.stop())

    emulator.start()


if __name__ == "__main__":
    main()
