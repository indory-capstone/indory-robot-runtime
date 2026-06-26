#!/usr/bin/env python3
"""Compatibility wrapper for ros_bridge.isaac_compat_gateway."""

from pathlib import Path
import sys


ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from ros_bridge.isaac_compat_gateway import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
