# Copyright 2026 Medrobots Engineering
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Read-only TeleDex WebSocket stream characterization; never publishes ROS."""

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import time
from typing import Callable, Dict, Optional, Sequence

from odometry_validation.teledex_adapter import TeleDexReferenceAdapter


def collect_stream_diagnostics(
        adapter: TeleDexReferenceAdapter, duration_s: float, poll_rate_hz: float,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep) -> Dict[str, object]:
    """Collect read-only transport evidence without creating a trial or ROS node."""
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("duration must be finite and positive")
    if not math.isfinite(poll_rate_hz) or poll_rate_hz <= 0.0:
        raise ValueError("poll rate must be finite and positive")
    interval_s = 1.0 / poll_rate_hz
    start = monotonic()
    valid_samples = 0
    invalid_samples = 0
    last_status = "no sample collected"
    adapter.start()
    try:
        while monotonic() - start < duration_s:
            sample = adapter.poll()
            if sample.valid_sample:
                valid_samples += 1
            else:
                invalid_samples += 1
            last_status = sample.status_reason
            remaining_s = duration_s - (monotonic() - start)
            if remaining_s > 0.0:
                sleep(min(interval_s, remaining_s))
        elapsed_s = max(0.0, monotonic() - start)
        return {
            "mode": "stationary_read_only_teledex_stream_diagnostic",
            "ros_command_publication": "none",
            "duration_s": elapsed_s,
            "poll_rate_hz": poll_rate_hz,
            "valid_poll_count": valid_samples,
            "invalid_poll_count": invalid_samples,
            "last_poll_status": last_status,
            "stream": asdict(adapter.stream_diagnostics()),
        }
    finally:
        adapter.stop()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only TeleDex stream diagnostic; does not publish ROS commands.")
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--poll-rate-hz", type=float, default=20.0)
    parser.add_argument(
        "--teledex-reference-root", default="/opt/teledex_reference")
    parser.add_argument(
        "--teledex-phone-to-base-rpy-deg", type=float, nargs=3,
        default=(-90.0, 0.0, 0.0), metavar=("ROLL", "PITCH", "YAW"))
    parser.add_argument(
        "--teledex-phone-to-base-translation-m", type=float, nargs=3,
        default=(0.1425, -0.1000, -0.0075), metavar=("X", "Y", "Z"))
    parser.add_argument("--teledex-stale-timeout-s", type=float, default=0.5)
    parser.add_argument(
        "--output", type=Path, required=True,
        help="New JSON evidence path; an existing file is never overwritten.")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    for name, value in (
            ("--duration-s", args.duration_s),
            ("--poll-rate-hz", args.poll_rate_hz),
            ("--teledex-stale-timeout-s", args.teledex_stale_timeout_s)):
        if not math.isfinite(value) or value <= 0.0:
            raise SystemExit(name + " must be finite and positive")
    if not all(math.isfinite(value) for value in (
            *args.teledex_phone_to_base_rpy_deg,
            *args.teledex_phone_to_base_translation_m)):
        raise SystemExit("TeleDex mount values must be finite")
    adapter = TeleDexReferenceAdapter(
        Path(args.teledex_reference_root),
        tuple(args.teledex_phone_to_base_rpy_deg),
        tuple(args.teledex_phone_to_base_translation_m),
        args.teledex_stale_timeout_s)
    result = collect_stream_diagnostics(
        adapter, args.duration_s, args.poll_rate_hz)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print("teledex_stream_diagnostic=" + str(args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
