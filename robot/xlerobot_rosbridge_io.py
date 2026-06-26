#!/usr/bin/env python3
"""Compatibility entrypoint for the fast ZMQ hardware I/O runtime."""

from pathlib import Path
import sys


ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from robot_io.xlerobot_fast_io import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
