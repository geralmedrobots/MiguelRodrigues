#!/usr/bin/env python3
# Copyright 2026 Medrobots
#
# Isolated bounded characterization: one stop-plus-FF-query per attempt.
# No automatic reconnect. No non-zero commands. Bounded in all I/O.
# Writes append-only JSONL evidence.
#
# Usage:
#   python3 roboteq_ff_query_characterization.py \
#       --port /dev/roboteq --baud 115200 \
#       --output /path/to/evidence.jsonl \
#       --operator <name>
#
# Preconditions (caller responsibility):
#   robot stationary; wheels isolated; E-stop accessible;
#   pharma-minimal-nodes.service inactive;
#   no other process owns /dev/roboteq or /dev/ttyUSB0;
#   ldd verified on pyserial before execution (N/A for pure-Python script,
#   pyserial itself has no native deps that require ldd; verified below).
#
# Abort conditions:
#   unexpected motion; non-zero command (never issued here);
#   port ownership conflict; startup-drain failure;
#   malformed command transmission; evidence-write failure;
#   unbounded wait (all I/O is bounded); automatic reconnect (never done here).

import argparse
import hashlib
import json
import os
import sys
import time
from typing import Any, Dict, List, Optional

try:
    import serial
    import serial.serialutil
except ImportError:
    print("ERROR: pyserial not available; install: pip3 install pyserial",
          file=sys.stderr)
    sys.exit(1)

# ── Constants matching production (harness settings) ─────────────────────────
STOP_COMMAND: bytes = b"!G 1 0\r!G 2 0\r!S 1 0\r!S 2 0\r"   # 28 bytes, 4 cmds
FF_QUERY: bytes = b"?FF\r"                                     # 4 bytes

EXPECTED_ACK_COUNT: int = 4
ACK_SEQ: tuple = (ord("+"), ord("\r"))                         # 0x2B 0x0D

DRAIN_QUIET_PERIOD_S: float = 0.100   # 100 ms — matches production default
DRAIN_ABSOLUTE_LIMIT_S: float = 0.250 # 250 ms — matches production default
DRAIN_MAX_BYTES: int = 256

ACK_TIMEOUT_S: float = 0.500          # 500 ms generous limit for 4 ACKs
POST_ACK_QUIET_S: float = 0.050       # 50 ms post-ACK quiet verification
QUERY_DEADLINE_S: float = 5.000       # 5 000 ms for FF response (vs 100 ms prod)
PRODUCTION_DEADLINE_S: float = 0.100  # 100 ms production reference threshold
MAX_RESPONSE_BYTES: int = 256

FF_RESPONSE_PREFIX: str = "FF="


# ── Timestamp helpers ─────────────────────────────────────────────────────────
def nsnow() -> int:
    return time.monotonic_ns()


def hex_str(b: bytes) -> str:
    return b.hex()


def raw_repr(b: bytes) -> str:
    """Printable with escape sequences; never silent about unexpected bytes."""
    return b.decode("latin-1").replace("\r", "\\r").replace("\n", "\\n")


# ── Startup drain ─────────────────────────────────────────────────────────────
def drain_startup_input(
    ser: serial.Serial,
    quiet_period_s: float = DRAIN_QUIET_PERIOD_S,
    absolute_limit_s: float = DRAIN_ABSOLUTE_LIMIT_S,
    max_bytes: int = DRAIN_MAX_BYTES,
) -> Dict[str, Any]:
    """
    Drain until quiet_period_s of silence or absolute_limit_s is exhausted.
    Faithfully replicates production RoboteqSerialTransport::drainStartupInput.
    Returns a dict suitable for JSONL embedding.
    """
    started_ns = nsnow()
    mono_start = time.monotonic()
    raw: bytearray = bytearray()
    last_byte_ns: Optional[int] = None
    delimiter_observed: bool = False

    quiet_deadline_mono = mono_start + quiet_period_s
    absolute_deadline_mono = mono_start + absolute_limit_s

    while time.monotonic() < absolute_deadline_mono:
        if len(raw) > max_bytes:
            return {
                "synchronized": False,
                "raw_bytes_hex": hex_str(bytes(raw)),
                "raw_bytes_raw": raw_repr(bytes(raw)),
                "byte_count": len(raw),
                "delimiter_observed": delimiter_observed,
                "started_ns": started_ns,
                "last_byte_ns": last_byte_ns,
                "completed_ns": nsnow(),
                "reason": "startup drain max bytes exceeded",
            }

        available = ser.in_waiting
        if available > 0:
            data = ser.read(available)
            raw.extend(data)
            last_byte_ns = nsnow()
            for b in data:
                if b in (0x0D, 0x0A):   # \r or \n
                    delimiter_observed = True
            # Reset quiet window on any byte
            quiet_deadline_mono = time.monotonic() + quiet_period_s
        else:
            time.sleep(0.001)

        if time.monotonic() >= quiet_deadline_mono:
            return {
                "synchronized": True,
                "raw_bytes_hex": hex_str(bytes(raw)),
                "raw_bytes_raw": raw_repr(bytes(raw)),
                "byte_count": len(raw),
                "delimiter_observed": delimiter_observed,
                "started_ns": started_ns,
                "last_byte_ns": last_byte_ns,
                "completed_ns": nsnow(),
                "reason": "",
            }

    return {
        "synchronized": False,
        "raw_bytes_hex": hex_str(bytes(raw)),
        "raw_bytes_raw": raw_repr(bytes(raw)),
        "byte_count": len(raw),
        "delimiter_observed": delimiter_observed,
        "started_ns": started_ns,
        "last_byte_ns": last_byte_ns,
        "completed_ns": nsnow(),
        "reason": "startup drain absolute limit exceeded",
    }


# ── ACK collection ────────────────────────────────────────────────────────────
def collect_acks(
    ser: serial.Serial,
    expected_count: int,
    timeout_s: float,
) -> Dict[str, Any]:
    """
    Collect exactly `expected_count` complete +\\r ACKs.
    Returns per-ACK timestamps and raw bytes.
    Any bytes after the last ACK in the same read batch are returned as
    'extra_after_acks' so the caller can handle them.
    """
    deadline_mono = time.monotonic() + timeout_s
    raw: bytearray = bytearray()
    ack_timestamps_ns: List[int] = []
    extra_after_acks: bytearray = bytearray()

    while time.monotonic() < deadline_mono:
        available = ser.in_waiting
        if available > 0:
            data = ser.read(available)
            for idx, byte in enumerate(data):
                if len(ack_timestamps_ns) >= expected_count:
                    # Already found all ACKs — remaining bytes are extra
                    extra_after_acks.extend(data[idx:])
                    break

                raw.append(byte)
                # An ACK is complete when the previous byte was '+' and this is '\r'
                if len(raw) >= 2 and raw[-2] == ACK_SEQ[0] and raw[-1] == ACK_SEQ[1]:
                    ack_timestamps_ns.append(nsnow())

                if len(ack_timestamps_ns) >= expected_count:
                    # Check remainder of this read batch
                    remainder_start = idx + 1
                    if remainder_start < len(data):
                        extra_after_acks.extend(data[remainder_start:])
                    break
        else:
            time.sleep(0.001)

        if len(ack_timestamps_ns) >= expected_count:
            break

    success = len(ack_timestamps_ns) >= expected_count
    reason = "" if success else (
        f"ACK timeout: collected {len(ack_timestamps_ns)} of {expected_count}"
    )

    return {
        "success": success,
        "ack_count": len(ack_timestamps_ns),
        "expected_ack_count": expected_count,
        "ack_timestamps_ns": ack_timestamps_ns,
        "raw_bytes_hex": hex_str(bytes(raw)),
        "raw_bytes_raw": raw_repr(bytes(raw)),
        "byte_count": len(raw),
        "extra_after_acks_hex": hex_str(bytes(extra_after_acks)),
        "extra_after_acks_raw": raw_repr(bytes(extra_after_acks)),
        "reason": reason,
    }


# ── Post-ACK quiet verification ───────────────────────────────────────────────
def verify_quiet(
    ser: serial.Serial,
    pre_buffered: bytes,
    quiet_period_s: float,
) -> Dict[str, Any]:
    """
    Verify that no bytes arrive for quiet_period_s.
    `pre_buffered` is any bytes already consumed after the last ACK
    that must be treated as extra (immediate failure).
    """
    if pre_buffered:
        return {
            "success": False,
            "extra_bytes_hex": hex_str(pre_buffered),
            "extra_bytes_raw": raw_repr(pre_buffered),
            "completed_ns": nsnow(),
            "reason": "bytes arrived immediately after last ACK (in same read batch)",
        }

    deadline_mono = time.monotonic() + quiet_period_s
    extra: bytearray = bytearray()

    while time.monotonic() < deadline_mono:
        available = ser.in_waiting
        if available > 0:
            extra.extend(ser.read(available))
            return {
                "success": False,
                "extra_bytes_hex": hex_str(bytes(extra)),
                "extra_bytes_raw": raw_repr(bytes(extra)),
                "completed_ns": nsnow(),
                "reason": "unexpected bytes during quiet verification",
            }
        time.sleep(0.001)

    return {
        "success": True,
        "extra_bytes_hex": "",
        "extra_bytes_raw": "",
        "completed_ns": nsnow(),
        "reason": "",
    }


# ── Line classifier ───────────────────────────────────────────────────────────
def classify_line(raw_line: bytes) -> str:
    """Classify a line received after ?FF\\r."""
    # Decode preserving all bytes (no UnicodeDecodeError)
    try:
        text = raw_line.rstrip(b"\r\n").decode("latin-1")
    except Exception:
        return "decode_error"

    if text.startswith(FF_RESPONSE_PREFIX):
        payload = text[len(FF_RESPONSE_PREFIX):]
        try:
            int(payload)
            return "expected_reply"
        except ValueError:
            return "malformed_ff_value"
    if raw_line.rstrip(b"\r\n") == b"+":
        return "acknowledgement"
    if raw_line.rstrip(b"\r\n") == b"-":
        return "rejection"
    if raw_line.rstrip(b"\r\n") == b"?FF":
        return "echo"
    return "unexpected_reply"


# ── FF query ──────────────────────────────────────────────────────────────────
def query_ff(
    ser: serial.Serial,
    deadline_s: float,
    prod_deadline_s: float = PRODUCTION_DEADLINE_S,
) -> Dict[str, Any]:
    """
    Transmit ?FF\\r and collect the response.
    Deadline is 5 000 ms; every received byte is timestamped.
    Records all 20 JSONL fields required by the characterization spec.
    Never reconnects or sends a second query.
    """
    write_started_ns = nsnow()

    # Write ?FF\r
    write_error: Optional[str] = None
    write_accepted_ns: Optional[int] = None
    write_fully_transmitted = False
    try:
        written = ser.write(FF_QUERY)
        write_accepted_ns = nsnow()
        write_fully_transmitted = (written == len(FF_QUERY))
        if not write_fully_transmitted:
            write_error = f"partial write: {written} of {len(FF_QUERY)} bytes"
    except Exception as ex:
        write_error = str(ex)

    deadline_mono = time.monotonic() + deadline_s
    raw: bytearray = bytearray()
    byte_timestamps_ns: List[int] = []
    first_byte_ns: Optional[int] = None
    complete_line_ns: Optional[int] = None
    received_lines: List[Dict[str, Any]] = []
    current_line: bytearray = bytearray()
    delayed_ack_received = False
    success = False
    ff_value: Optional[int] = None
    final_reason = ""

    if write_error:
        final_reason = f"write error: {write_error}"
        return _ff_result(
            write_started_ns, write_accepted_ns, write_fully_transmitted, write_error,
            raw, byte_timestamps_ns, first_byte_ns, complete_line_ns,
            received_lines, current_line, success, ff_value, final_reason,
            deadline_s, prod_deadline_s, delayed_ack_received, ser.in_waiting,
        )

    while time.monotonic() < deadline_mono:
        available = ser.in_waiting
        if available > 0:
            data = ser.read(available)
            for byte in data:
                raw.append(byte)
                ts = nsnow()
                byte_timestamps_ns.append(ts)

                if first_byte_ns is None:
                    first_byte_ns = ts

                current_line.append(byte)

                if byte in (0x0D, 0x0A):
                    # Complete line received
                    complete_line_ns = ts
                    line_text = bytes(current_line).rstrip(b"\r\n").decode("latin-1")
                    classification = classify_line(bytes(current_line))

                    elapsed_from_write_start_ns = ts - write_started_ns
                    elapsed_from_write_accepted_ns = (
                        ts - write_accepted_ns if write_accepted_ns else None
                    )

                    line_record: Dict[str, Any] = {
                        "raw_hex": hex_str(bytes(current_line)),
                        "raw_repr": raw_repr(bytes(current_line)),
                        "text": line_text,
                        "classification": classification,
                        "complete_line_ns": ts,
                        "latency_from_write_start_ns": elapsed_from_write_start_ns,
                        "latency_from_write_accepted_ns": elapsed_from_write_accepted_ns,
                        "within_production_deadline": (
                            elapsed_from_write_start_ns / 1e6 <= prod_deadline_s * 1000
                        ),
                    }
                    received_lines.append(line_record)

                    if classification == "acknowledgement":
                        # Unexpected ACK during query — record, continue reading
                        delayed_ack_received = True
                        current_line = bytearray()
                        continue

                    if classification == "expected_reply":
                        ff_value = int(line_text[len(FF_RESPONSE_PREFIX):])
                        success = True
                        current_line = bytearray()
                        # Drain any extra bytes (bounded)
                        extra_after: bytearray = bytearray()
                        extra_deadline_mono = time.monotonic() + 0.050
                        while time.monotonic() < extra_deadline_mono:
                            if ser.in_waiting > 0:
                                extra_after.extend(ser.read(ser.in_waiting))
                            else:
                                time.sleep(0.001)
                        raw.extend(extra_after)
                        for _ in extra_after:
                            byte_timestamps_ns.append(nsnow())
                        final_reason = "" if not extra_after else (
                            f"extra bytes after FF response: {hex_str(bytes(extra_after))}"
                        )
                        break

                    if classification in (
                        "rejection", "malformed_ff_value", "unexpected_reply", "echo",
                    ):
                        final_reason = f"parser rejection: {classification}"
                        current_line = bytearray()
                        # Do NOT reconnect; keep reading in case correct line follows
                        continue

                    current_line = bytearray()

                if len(raw) > MAX_RESPONSE_BYTES:
                    final_reason = "response exceeded max bytes"
                    break

            if success or "exceeded" in final_reason:
                break
        else:
            time.sleep(0.001)

    if not success and not final_reason:
        if bytes(current_line):
            final_reason = "diagnostic query timed out with a partial response"
        else:
            final_reason = "diagnostic query timed out"

    return _ff_result(
        write_started_ns, write_accepted_ns, write_fully_transmitted, write_error,
        raw, byte_timestamps_ns, first_byte_ns, complete_line_ns,
        received_lines, current_line, success, ff_value, final_reason,
        deadline_s, prod_deadline_s, delayed_ack_received, ser.in_waiting,
    )


def _ff_result(
    write_started_ns: int,
    write_accepted_ns: Optional[int],
    write_fully_transmitted: bool,
    write_error: Optional[str],
    raw: bytearray,
    byte_timestamps_ns: List[int],
    first_byte_ns: Optional[int],
    complete_line_ns: Optional[int],
    received_lines: List[Dict[str, Any]],
    partial_current_line: bytearray,
    success: bool,
    ff_value: Optional[int],
    reason: str,
    deadline_s: float,
    prod_deadline_s: float,
    delayed_ack_received: bool,
    buffered_at_end: int,
) -> Dict[str, Any]:
    """Assemble the full FF result dict with all 20 characterization fields."""
    query_latency_ns: Optional[int] = None
    response_within_production_deadline = False
    response_after_production_deadline = False

    if success and complete_line_ns is not None:
        query_latency_ns = complete_line_ns - write_started_ns
        latency_ms = query_latency_ns / 1e6
        response_within_production_deadline = latency_ms <= prod_deadline_s * 1000
        response_after_production_deadline = latency_ms > prod_deadline_s * 1000
    elif first_byte_ns is not None:
        query_latency_ns = first_byte_ns - write_started_ns

    return {
        # Field 6-8: Query transmission
        "query_transmitted_raw": raw_repr(FF_QUERY),
        "query_transmitted_hex": hex_str(FF_QUERY),
        "write_started_ns": write_started_ns,
        "write_accepted_ns": write_accepted_ns,
        "write_fully_transmitted": write_fully_transmitted,
        "write_error": write_error,
        # Field 9-10: Received bytes
        "all_received_bytes_hex": hex_str(bytes(raw)),
        "all_received_bytes_raw": raw_repr(bytes(raw)),
        "received_byte_count": len(raw),
        "byte_timestamps_ns": byte_timestamps_ns,
        # Field 10: First-byte timestamp
        "first_byte_ns": first_byte_ns,
        # Field 11: Complete-line timestamp
        "complete_line_ns": complete_line_ns,
        # Field 12: Per-line classifications
        "received_lines": received_lines,
        # Field 13: Partial trailing bytes
        "partial_trailing_hex": hex_str(bytes(partial_current_line)),
        "partial_trailing_raw": raw_repr(bytes(partial_current_line)),
        # Field 14: State before completion/timeout
        "framing_state_before_timeout": (
            "partial_line_buffered" if partial_current_line else "synchronized"
        ),
        # Field 15: Framing state
        "framing_state": (
            "synchronized" if success and not partial_current_line
            else "partial_line" if partial_current_line
            else "no_bytes_received" if not raw
            else "unresolved"
        ),
        # Field 16: Failure reason
        "reason": reason,
        # Field 17: Query latency
        "query_latency_ns": query_latency_ns,
        "query_latency_ms": round(query_latency_ns / 1e6, 3) if query_latency_ns else None,
        # Field 18: Delayed ACK
        "delayed_ack_received": delayed_ack_received,
        # Field 19: Response timing relative to production deadline
        "response_within_production_100ms": response_within_production_deadline,
        "response_after_production_100ms": response_after_production_deadline,
        # Field 20: Buffered bytes when attempt ended
        "buffered_bytes_at_end": buffered_at_end,
        # Summary flags
        "success": success,
        "ff_value": ff_value,
    }


# ── Single attempt ────────────────────────────────────────────────────────────
def run_attempt(port: str, baud: int, attempt_num: int) -> Dict[str, Any]:
    """
    One complete attempt:
      1. Open port
      2. Startup drain (bounded)
      3. Send stop: !G 1 0\\r!G 2 0\\r!S 1 0\\r!S 2 0\\r
      4. Collect exactly 4 +\\r ACKs
      5. Quiet verification (bounded)
      6. Send ?FF\\r
      7. Collect response (5 000 ms deadline, no reconnect)
      8. Close port
    """
    attempt_started_ns = nsnow()

    # ── 1. Open ──
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,           # Non-blocking reads
            write_timeout=0.5,   # Bounded write
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
    except Exception as ex:
        return {
            "type": "attempt",
            "attempt": attempt_num,
            "attempt_started_ns": attempt_started_ns,
            "port_opened": False,
            "abort": True,
            "abort_reason": f"port open failed: {ex}",
        }

    try:
        # ── 2. Startup drain ──
        drain = drain_startup_input(ser)
        if not drain["synchronized"]:
            return {
                "type": "attempt",
                "attempt": attempt_num,
                "attempt_started_ns": attempt_started_ns,
                "port_opened": True,
                "drain": drain,
                "abort": True,
                "abort_reason": f"startup drain failed: {drain['reason']}",
            }

        # ── 3. Send stop command ──
        stop_write_started_ns = nsnow()
        try:
            written = ser.write(STOP_COMMAND)
            stop_write_accepted_ns = nsnow()
            stop_write_fully_accepted = (written == len(STOP_COMMAND))
        except Exception as ex:
            return {
                "type": "attempt",
                "attempt": attempt_num,
                "attempt_started_ns": attempt_started_ns,
                "port_opened": True,
                "drain": drain,
                "abort": True,
                "abort_reason": f"stop write exception: {ex}",
            }

        if not stop_write_fully_accepted:
            return {
                "type": "attempt",
                "attempt": attempt_num,
                "attempt_started_ns": attempt_started_ns,
                "port_opened": True,
                "drain": drain,
                "abort": True,
                "abort_reason": (
                    f"partial stop write: {written} of {len(STOP_COMMAND)} bytes"
                ),
            }

        stop_cmd_record = {
            "transmitted_raw": raw_repr(STOP_COMMAND),
            "transmitted_hex": hex_str(STOP_COMMAND),
            "byte_count": len(STOP_COMMAND),
            "command_count": 4,
            "write_started_ns": stop_write_started_ns,
            "write_accepted_ns": stop_write_accepted_ns,
            "write_fully_accepted": stop_write_fully_accepted,
        }

        # ── 4. Collect 4 ACKs ──
        ack_result = collect_acks(ser, EXPECTED_ACK_COUNT, ACK_TIMEOUT_S)
        if not ack_result["success"]:
            return {
                "type": "attempt",
                "attempt": attempt_num,
                "attempt_started_ns": attempt_started_ns,
                "port_opened": True,
                "drain": drain,
                "stop_command": stop_cmd_record,
                "ack_collection": ack_result,
                "abort": True,
                "abort_reason": f"ACK collection failed: {ack_result['reason']}",
            }

        # ── 5. Post-ACK quiet verification ──
        extra_bytes = bytes.fromhex(ack_result["extra_after_acks_hex"])
        quiet_result = verify_quiet(ser, extra_bytes, POST_ACK_QUIET_S)
        if not quiet_result["success"]:
            return {
                "type": "attempt",
                "attempt": attempt_num,
                "attempt_started_ns": attempt_started_ns,
                "port_opened": True,
                "drain": drain,
                "stop_command": stop_cmd_record,
                "ack_collection": ack_result,
                "quiet_verification": quiet_result,
                "abort": True,
                "abort_reason": f"quiet verification failed: {quiet_result['reason']}",
            }

        # ── 6-10. FF query ──
        ff_result = query_ff(ser, QUERY_DEADLINE_S)

        attempt_completed_ns = nsnow()
        attempt_duration_ms = (attempt_completed_ns - attempt_started_ns) / 1e6

        return {
            "type": "attempt",
            "attempt": attempt_num,
            "attempt_started_ns": attempt_started_ns,
            "attempt_completed_ns": attempt_completed_ns,
            "attempt_duration_ms": round(attempt_duration_ms, 3),
            "port_opened": True,
            "abort": False,
            "drain": drain,
            "stop_command": stop_cmd_record,
            "ack_collection": ack_result,
            "quiet_verification": quiet_result,
            "ff_query": ff_result,
        }

    finally:
        try:
            ser.close()
        except Exception:
            pass


# ── Post-analysis ─────────────────────────────────────────────────────────────
def analyze_results(results: List[Dict[str, Any]]) -> Dict[str, Any]:
    valid = [r for r in results if r.get("type") == "attempt" and not r.get("abort")]
    ff_list = [r["ff_query"] for r in valid if "ff_query" in r]

    successes = [f for f in ff_list if f.get("success")]
    within_100ms = [f for f in successes if f.get("response_within_production_100ms")]
    after_100ms = [f for f in successes if f.get("response_after_production_100ms")]
    no_response = [f for f in ff_list if not f.get("success") and not f.get("first_byte_ns")]
    partial = [
        f for f in ff_list
        if not f.get("success") and f.get("first_byte_ns") and not f.get("complete_line_ns")
    ]

    latencies_ns = [
        f["query_latency_ns"]
        for f in successes
        if f.get("query_latency_ns") is not None
    ]
    latencies_ms = sorted(f / 1e6 for f in latencies_ns)
    n = len(latencies_ms)

    def pct(lst: List[float], p: float) -> Optional[float]:
        if not lst:
            return None
        idx = min(int(n * p), n - 1)
        return round(lst[idx], 3)

    ff_transmitted_all = all(f.get("write_fully_transmitted") for f in ff_list)
    delayed_acks_any = any(f.get("delayed_ack_received") for f in ff_list)

    unexpected_lines: List[Dict[str, Any]] = []
    for f in ff_list:
        for line in f.get("received_lines", []):
            if line.get("classification") not in ("expected_reply", "echo"):
                unexpected_lines.append(line)

    # Root-cause classification (Requirement: Agent 3 final word, but first pass here)
    if not ff_list:
        classification = "inconclusive"
        classification_detail = "all attempts aborted before FF query"
    elif not ff_transmitted_all:
        classification = "query_not_transmitted"
        classification_detail = "?FF\\r write failed on one or more attempts"
    elif no_response and len(no_response) == len(ff_list):
        classification = "no_controller_response"
        classification_detail = "?FF\\r transmitted every attempt; device silent in all"
    elif after_100ms and not within_100ms:
        classification = "delayed_controller_response"
        classification_detail = "responses received only after 100 ms production boundary"
    elif after_100ms:
        classification = "delayed_controller_response"
        classification_detail = f"{len(after_100ms)} of {len(ff_list)} after 100ms"
    elif successes and len(successes) == len(ff_list):
        classification = "within_deadline"
        classification_detail = "all responses within 100 ms; production deadline is fine"
    elif [f for f in ff_list if f.get("reason", "").startswith("parser rejection")]:
        classification = "parser_rejection"
        classification_detail = "device responded but line rejected by parser"
    elif delayed_acks_any:
        classification = "ownership_contamination"
        classification_detail = "delayed +\\r ACKs arrived during FF query window"
    elif partial:
        classification = "no_controller_response"
        classification_detail = "partial response(s) only — no complete line delivered"
    else:
        classification = "inconclusive"
        classification_detail = "mixed results; further investigation required"

    deadline_too_short = len(after_100ms) > 0
    recommend_deadline = "extend to >=5000ms" if deadline_too_short else "100ms appears sufficient"

    return {
        "type": "analysis",
        "completed_attempts": len(valid),
        "ff_queries_issued": len(ff_list),
        "successes": len(successes),
        "within_production_100ms": len(within_100ms),
        "after_production_100ms": len(after_100ms),
        "no_response_at_all": len(no_response),
        "partial_response": len(partial),
        "ff_transmitted_every_attempt": ff_transmitted_all,
        "delayed_ack_contamination_observed": delayed_acks_any,
        "unexpected_lines_count": len(unexpected_lines),
        "unexpected_lines": unexpected_lines,
        "latency_min_ms": round(min(latencies_ms), 3) if latencies_ms else None,
        "latency_median_ms": pct(latencies_ms, 0.50),
        "latency_p95_ms": pct(latencies_ms, 0.95),
        "latency_max_ms": round(max(latencies_ms), 3) if latencies_ms else None,
        "classification": classification,
        "classification_detail": classification_detail,
        "production_100ms_deadline_too_short": deadline_too_short,
        "deadline_recommendation": recommend_deadline,
    }


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Isolated stop+?FF characterization. "
            "10 attempts, each with fresh port open/close. "
            "No reconnect loop."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", default="/dev/roboteq", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--output", required=True, help="Append-only JSONL evidence file")
    parser.add_argument("--operator", default="unknown", help="Operator name")
    parser.add_argument("--attempts", type=int, default=10, help="Number of attempts (default 10)")
    args = parser.parse_args()

    # ── Evidence file directory check ──
    evidence_dir = os.path.dirname(args.output)
    if evidence_dir and not os.path.isdir(evidence_dir):
        print(f"ERROR: evidence directory does not exist: {evidence_dir}", file=sys.stderr)
        sys.exit(1)

    print("=" * 70)
    print("roboteq_ff_query_characterization")
    print(f"  port={args.port}  baud={args.baud}  operator={args.operator}")
    print(f"  output={args.output}")
    print(f"  stop_cmd=({len(STOP_COMMAND)} bytes) {STOP_COMMAND!r}")
    print(f"  ff_deadline={QUERY_DEADLINE_S * 1000:.0f} ms")
    print(f"  production_deadline={PRODUCTION_DEADLINE_S * 1000:.0f} ms (reference only)")
    print("=" * 70)

    results: List[Dict[str, Any]] = []
    any_abort = False

    for i in range(1, args.attempts + 1):
        print(f"\n[attempt {i}/{args.attempts}]")
        r = run_attempt(args.port, args.baud, i)
        results.append(r)

        # Append-only, immediate write with fsync
        try:
            with open(args.output, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(r, separators=(",", ":")) + "\n")
                fh.flush()
                os.fsync(fh.fileno())
        except Exception as ex:
            print(f"ABORT: evidence write failed: {ex}", file=sys.stderr)
            sys.exit(1)

        if r.get("abort"):
            print(f"  ABORT: {r.get('abort_reason')}")
            any_abort = True
            break

        drain = r["drain"]
        stop = r["stop_command"]
        acks = r["ack_collection"]
        quiet = r["quiet_verification"]
        ff = r["ff_query"]

        print(f"  drain: ok={drain['synchronized']}  bytes={drain['byte_count']}")
        print(f"  stop:  accepted={stop['write_fully_accepted']}  "
              f"acks={acks['ack_count']}/{acks['expected_ack_count']}")
        print(f"  quiet: ok={quiet['success']}")
        print(f"  ff:    transmitted={ff['write_fully_transmitted']}  "
              f"success={ff['success']}  value={ff['ff_value']}  "
              f"latency={ff.get('query_latency_ms','n/a')} ms")
        if not ff["success"]:
            print(f"  ff:    reason={ff['reason']!r}")
            rx_hex = ff.get("all_received_bytes_hex", "")
            if rx_hex:
                print(f"  ff:    received_bytes_hex={rx_hex}  "
                      f"raw={ff.get('all_received_bytes_raw','')!r}")
            else:
                print("  ff:    NO bytes received after ?FF\\r")
        if ff.get("delayed_ack_received"):
            print("  ff:    WARNING: delayed +\\r ACK received during FF query window")

    # ── Analysis ──
    analysis = analyze_results(results)
    try:
        with open(args.output, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(analysis, separators=(",", ":")) + "\n")
            fh.flush()
            os.fsync(fh.fileno())
    except Exception as ex:
        print(f"ABORT: analysis write failed: {ex}", file=sys.stderr)
        sys.exit(1)

    print("\n" + "=" * 70)
    print("ANALYSIS")
    print("=" * 70)
    for k, v in analysis.items():
        if k not in ("type", "unexpected_lines"):
            print(f"  {k}: {v}")
    if analysis["unexpected_lines"]:
        print("  Unexpected lines:")
        for ul in analysis["unexpected_lines"]:
            print(f"    [{ul['classification']}] {ul['raw_repr']!r}  "
                  f"latency={ul.get('latency_from_write_start_ns','?')} ns")

    # ── SHA-256 ──
    sha256 = hashlib.sha256()
    with open(args.output, "rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            sha256.update(chunk)
    digest = sha256.hexdigest()
    print(f"\nEvidence: {args.output}")
    print(f"SHA-256:  {digest}")

    sys.exit(1 if any_abort else 0)


if __name__ == "__main__":
    main()
