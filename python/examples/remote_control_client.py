#!/usr/bin/env python3
"""Compatibility wrapper for mjctl; see mjctl --help for usage."""

from xbot2_py_mujoco.mjctl import main


if __name__ == "__main__":
    raise SystemExit(main())
