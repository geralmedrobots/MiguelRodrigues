#!/usr/bin/env python3
"""Stop the active SLAM v1 recorder; the recorder performs finalization."""
import json
from pathlib import Path
import os
import signal
import sys

STATE_FILE = Path("/tmp/pharmarobot_slam_v1_trial.json")


def main():
    if not STATE_FILE.exists():
        print("No active SLAM v1 trial", file=sys.stderr)
        return 1
    state = json.loads(STATE_FILE.read_text())
    os.kill(int(state["pid"]), signal.SIGINT)
    print(state["directory"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
