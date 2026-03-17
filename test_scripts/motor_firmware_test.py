#!/usr/bin/env python3
"""
Minimal serial-hardware test harness with expected-pass / expected-fail.

Usage:
    python hw_tests.py                 # interactive serial select; run all tests
    python hw_tests.py --list          # list discovered tests
    python hw_tests.py --pattern echo  # run tests matching 'echo'
    python hw_tests.py --baud 115200   # set baud
"""

from __future__ import annotations

import argparse
import sys
import time
import re
import traceback
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

import serial
import serial.tools.list_ports


# =========================
# Registry and decorators
# =========================

_TEST_REGISTRY: Dict[str, Tuple[Callable, str]] = {}


def _register_test(func: Callable, expectation: str) -> Callable:
    """
    Register a test function with an expectation.

    expectation: "pass" or "fail"
    """
    name = func.__name__
    if not name.startswith("test_"):
        raise ValueError("Test functions must start with 'test_'.")
    _TEST_REGISTRY[name] = (func, expectation)
    return func


def expect_pass(func: Callable) -> Callable:
    """Decorator: test is expected to pass."""
    return _register_test(func, "pass")


def expect_fail(func: Callable) -> Callable:
    """Decorator: test is expected to fail (e.g., known bug or unimplemented)."""
    return _register_test(func, "fail")


# =========================
# Exceptions and results
# =========================

class TestFailure(AssertionError):
    """Raise this to mark an assertion-style failure."""


@dataclass
class TestResult:
    name: str
    expectation: str  # "pass" or "fail"
    status: str       # "PASS", "FAIL", "XPASS", "XFAIL", "SKIP"
    duration_s: float
    error: Optional[str] = None


# =========================
# Serial utilities + context
# =========================

def choose_serial_port(baudrate: int = 9600, timeout: float = 1.0) -> serial.Serial:
    """
    List available serial ports, prompt the user to select one,
    then open and return a serial connection.
    """
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found.")

    print("Available serial ports:")
    for i, port in enumerate(ports, start=1):
        print(f"{i}: {port.device} - {port.description}")

    while True:
        try:
            choice = int(input("Select a port by number: ").strip())
            if 1 <= choice <= len(ports):
                port_name = ports[choice - 1].device
                break
            print("Invalid selection. Try again.")
        except ValueError:
            print("Please enter a number.")

    conn = serial.Serial(port=port_name, baudrate=baudrate, timeout=timeout)
    print(f"Opened {port_name} at {baudrate} baud.")
    return conn


class TestContext:
    """
    Per-run context injected into each test.

    Provides:
        - serial: the active serial.Serial connection
        - prompt(): manual user prompts
        - sleep(): wall-clock sleep using time.monotonic
        - write_line() / read_line() convenience
        - reconnect(): close/open port (e.g., after power cycle)
        - wait_for(): polling helper with timeout
    """

    def __init__(self, serial_conn: serial.Serial):
        self.serial = serial_conn

    def prompt(self, message: str, require_yes: bool = False) -> bool:
        """
        Prompt the user to perform a manual action.
        Returns True if the user pressed Enter, or answered yes when required.
        """
        if require_yes:
            ans = input(f"{message} (type 'yes' to continue): ").strip().lower()
            return ans == "yes"
        input(f"{message} Press Enter when ready...")
        return True

    @staticmethod
    def sleep(seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            time.sleep(min(0.05, end - time.monotonic()))

    def write_line(self, text: str) -> None:
        if not text.endswith("\n"):
            text = text + "\n"
        self.serial.write(text.encode("utf-8"))
        self.serial.flush()

    def read_line(self, timeout_s: float = 2.0) -> Optional[str]:
        """
        Read one line with a temporary timeout override.
        Returns a decoded line without trailing newline, or None on timeout.
        """
        old = self.serial.timeout
        self.serial.timeout = timeout_s
        try:
            raw = self.serial.readline()
        finally:
            self.serial.timeout = old
        if not raw:
            return None
        return raw.decode("utf-8", errors="replace").rstrip("\r\n")

    def wait_for(
        self,
        predicate: Callable[[], bool],
        timeout_s: float,
        poll_s: float = 0.1,
    ) -> bool:
        """
        Poll predicate() until True or timeout.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            self.sleep(poll_s)
        return False

    def reconnect(self) -> None:
        """
        Close and reopen the current port with the same settings.
        Useful after a power cycle or USB reconnection.
        """
        port = self.serial.port
        baud = self.serial.baudrate
        tout = self.serial.timeout
        self.serial.close()
        self.serial = serial.Serial(port=port, baudrate=baud, timeout=tout)
        self.sleep(0.2)

    def send_cmd(self, text: str, eol: str = "\r\n") -> None:
        """
        Send a command terminated by carriage return (default).
        """
        payload = (text + eol).encode("utf-8")
        self.serial.write(payload)
        self.serial.flush()

    def _is_cmd_result_line(self, line: str) -> bool:
        """
        Return True if 'line' looks like a command result token.
        Here we treat 'OK' and '!' as terminal results.
        """
        return line in {"OK", "!"}

    def read_cmd_result(self, timeout_s: float = 3.0) -> Optional[str]:
        """
        Read lines, skipping status chatter, until a command result is seen
        or timeout expires. Returns 'OK', '!' or None.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            line = self.read_line(timeout_s=0.5)
            if line is None:
                continue
            if self._is_cmd_result_line(line):
                return line
            # Otherwise: ignore status/telemetry lines
        return None


# =========================
# Test runner
# =========================

def _run_one(
    name: str,
    func: Callable[[TestContext], None],
    expectation: str,
    ctx: TestContext,
) -> TestResult:
    start = time.monotonic()
    try:
        func(ctx)
    except Exception as exc:
        duration = time.monotonic() - start
        tb = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
        if expectation == "fail":
            # Expected to fail: this is good (XFAIL)
            return TestResult(
                name=name,
                expectation=expectation,
                status="XFAIL",
                duration_s=duration,
                error=tb,
            )
        # Unexpected failure
        return TestResult(
            name=name,
            expectation=expectation,
            status="FAIL",
            duration_s=duration,
            error=tb,
        )

    duration = time.monotonic() - start
    if expectation == "fail":
        # Test was expected to fail but passed
        return TestResult(
            name=name,
            expectation=expectation,
            status="XPASS",
            duration_s=duration,
        )
    return TestResult(
        name=name,
        expectation=expectation,
        status="PASS",
        duration_s=duration,
    )


def run_tests(
    ctx: TestContext,
    pattern: Optional[str] = None,
) -> List[TestResult]:
    """
    Run tests in the registry, optionally filtering by substring pattern.
    """
    selected = []
    for name, (func, exp) in sorted(_TEST_REGISTRY.items()):
        if pattern and pattern.lower() not in name.lower():
            continue
        selected.append((name, func, exp))

    if not selected:
        print("No tests selected.")
        return []

    results: List[TestResult] = []
    print(f"Running {len(selected)} test(s)...")
    for name, func, exp in selected:
        print(f"- {name} [{exp}]")
        res = _run_one(name, func, exp, ctx)
        results.append(res)

    # Summary
    print("\nResults:")
    cats = {"PASS": 0, "FAIL": 0, "XFAIL": 0, "XPASS": 0, "SKIP": 0}
    for r in results:
        cats[r.status] = cats.get(r.status, 0) + 1
        line = f"{r.status:5s} {r.name} ({r.duration_s:.2f}s)"
        print(line)
        if r.error and r.status in {"FAIL", "XFAIL"}:
            print("  └─ Traceback (trimmed):")
            print("\n".join("    " + ln for ln in r.error.strip().splitlines()[-8:]))

    total = sum(cats.values())
    print(
        f"\nSummary: {total} run  "
        f"PASS={cats['PASS']}  FAIL={cats['FAIL']}  "
        f"XFAIL={cats['XFAIL']}  XPASS={cats['XPASS']}"
    )
    return results

# =========================
# Test Helpers
# =========================
# --- Helpers for SHOW parsing (add once near your other tests) ---
def reset_to_defaults(ctx: TestContext, verify: bool = True, reboot: bool = False) -> None:
    """
    Restore factory defaults so the unit is ready for installation.

    Steps:
      - RESETCONFIG and expect OK
      - (optional) Verify defaults via SHOW
      - (optional) Power-cycle prompt + re-verify

    Designed to be safe to call at the end of a run, even after failures.
    """
    try:
        ctx.send_cmd("RESETCONFIG")
        tok = ctx.read_cmd_result(timeout_s=5.0)
        if tok != "OK":
            print("Warning: RESETCONFIG did not return OK; continuing.", file=sys.stderr)
            verify = False  # avoid misleading verify if reset failed
    except Exception as exc:
        print(f"Warning: RESETCONFIG raised {exc!r}.", file=sys.stderr)
        return

    if verify:
        try:
            lines = _read_show_until_ok(ctx, timeout_s=5.0)
            sp = _parse_setpoint_rpm(lines)
            kp = _parse_pid_value(lines, "Kp")
            ki = _parse_pid_value(lines, "Ki")
            kd = _parse_pid_value(lines, "Kd")
            cutoff_enabled = _parse_cutoff_enabled(lines)
            autorestart_enabled = _parse_auto_restart_enabled(lines)
            limit_ma = _parse_current_limit_ma(lines)
            log_head = _parse_label_int(lines, "Log Head Index")

            problems = []
            if sp != 210:
                problems.append(f"Setpoint RPM {sp} != 210")
            if abs(kp - 0.02) > 1e-6:
                problems.append(f"Kp {kp} != 0.02")
            if abs(ki - 0.05) > 1e-6:
                problems.append(f"Ki {ki} != 0.05")
            if abs(kd - 0.00) > 1e-6:
                problems.append(f"Kd {kd} != 0.00")
            if not cutoff_enabled:
                problems.append("Current Cutoff not ENABLED")
            if not autorestart_enabled:
                problems.append("Auto Restart not ENABLED")
            if limit_ma != 550:
                problems.append(f"Current Limit {limit_ma} mA != 550 mA")
            if log_head != 0:
                problems.append(f"Log Head Index {log_head} != 0")

            if problems:
                print("Warning: Post-reset verify mismatches: " + "; ".join(problems), file=sys.stderr)
        except Exception as exc:
            print(f"Warning: SHOW verify after RESETCONFIG failed: {exc!r}", file=sys.stderr)

    if reboot:
        # Give operator a clean, persisted state
        try:
            ctx.serial.close()
        except Exception:
            pass
        ctx.prompt("Power-cycle the unit (OFF → ON). Press Enter when back on.")
        ctx.sleep(2.0)
        ctx.reconnect()
        if verify:
            try:
                lines2 = _read_show_until_ok(ctx, timeout_s=5.0)
                sp2 = _parse_setpoint_rpm(lines2)
                if sp2 != 210:
                    print("Warning: Post-reboot setpoint not at default 210.", file=sys.stderr)
            except Exception as exc:
                print(f"Warning: Post-reboot verify failed: {exc!r}", file=sys.stderr)

def _read_show_until_ok(ctx: TestContext, timeout_s: float = 5.0) -> List[str]:
    ctx.send_cmd("SHOW")
    lines: List[str] = []
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = ctx.read_line(timeout_s=0.6)
        if line is None:
            continue
        if line == "OK":
            return lines
        lines.append(line)
    raise TestFailure("Timed out waiting for 'OK' after SHOW.")

def _parse_pid_value(lines: List[str], label: str) -> float:
    target = next((ln for ln in lines if ln.strip().startswith(f"{label}:")), None)
    if target is None:
        raise TestFailure(f"SHOW output missing '{label}:' line.")
    m = re.search(rf"{re.escape(label)}:\s*([+-]?\d+(?:\.\d+)?)", target)
    if not m:
        raise TestFailure(f"Could not parse {label} from: {target!r}")
    return float(m.group(1))


def _parse_current_limit_ma(lines: List[str]) -> int:
    """
    From SHOW output lines, extract 'Current Limit: <int> mA' as an int (mA).
    """
    target = next(
        (ln for ln in lines if ln.strip().startswith("Current Limit:")),
        None,
    )
    if target is None:
        raise TestFailure("SHOW output missing 'Current Limit:' line.")
    m = re.search(r"Current\s+Limit:\s*(\d+)\s*mA", target)
    if not m:
        raise TestFailure(f"Could not parse Current Limit from: {target!r}")
    return int(m.group(1))


def _parse_cutoff_enabled(lines: List[str]) -> bool:
    """
    From SHOW output lines, return True if 'Current Cutoff: ENABLED',
    False if 'Current Cutoff: DISABLED'. Raises on missing/unknown.
    """
    target = next(
        (ln for ln in lines if ln.strip().startswith("Current Cutoff:")),
        None,
    )
    if target is None:
        raise TestFailure("SHOW output missing 'Current Cutoff:' line.")
    m = re.search(r"Current\s+Cutoff:\s*(ENABLED|DISABLED)", target, re.IGNORECASE)
    if not m:
        raise TestFailure(f"Could not parse cutoff state from: {target!r}")
    return m.group(1).upper() == "ENABLED"


def _parse_auto_restart_enabled(lines: List[str]) -> bool:
    """
    From SHOW output lines, return True if 'Auto Restart: ENABLED',
    False if 'Auto Restart: DISABLED'. Raises on missing/unknown.
    """
    target = next(
        (ln for ln in lines if ln.strip().startswith("Auto Restart:")),
        None,
    )
    if target is None:
        raise TestFailure("SHOW output missing 'Auto Restart:' line.")
    m = re.search(r"Auto\s+Restart:\s*(ENABLED|DISABLED)", target, re.IGNORECASE)
    if not m:
        raise TestFailure(f"Could not parse Auto Restart state from: {target!r}")
    return m.group(1).upper() == "ENABLED"

def _parse_setpoint_rpm(lines: List[str]) -> int:
    target = next((ln for ln in lines if "Setpoint RPM:" in ln), None)
    if target is None:
        raise TestFailure("SHOW output missing 'Setpoint RPM:' line.")
    m = re.search(r"Setpoint\s+RPM:\s*(\d+)", target)
    if not m:
        raise TestFailure(f"Could not parse Setpoint RPM from: {target!r}")
    return int(m.group(1))

def _parse_label_int(lines: List[str], label: str) -> int:
    target = next((ln for ln in lines if ln.strip().startswith(f"{label}:")), None)
    if target is None:
        raise TestFailure(f"SHOW output missing '{label}:' line.")
    m = re.search(rf"{re.escape(label)}:\s*(\d+)", target)
    if not m:
        raise TestFailure(f"Could not parse integer for {label} from: {target!r}")
    return int(m.group(1))

# =========================
# Tests
# =========================

# ---- SETRPM ----

@expect_pass
def test_setrpm_no_args(ctx: TestContext) -> None:
    """
    'SETRPM' with no arguments -> device should respond '!' (too few args).
    """
    ctx.send_cmd("SETRPM")
    result = ctx.read_cmd_result(timeout_s=3.0)
    if result != "!":
        raise TestFailure(f"Expected '!' for too few args, got: {result!r}")


@expect_pass
def test_setrpm_too_many_args_two(ctx: TestContext) -> None:
    """
    'SETRPM 100 200' -> '!' (too many args).
    """
    ctx.send_cmd("SETRPM 100 200")
    result = ctx.read_cmd_result(timeout_s=3.0)
    if result != "!":
        raise TestFailure(f"Expected '!' for too many args, got: {result!r}")


@expect_pass
def test_setrpm_below_min(ctx: TestContext) -> None:
    """
    'SETRPM 20' -> '!' (below valid range).
    """
    ctx.send_cmd("SETRPM 20")
    result = ctx.read_cmd_result(timeout_s=3.0)
    if result != "!":
        raise TestFailure(f"Expected '!' for below-range RPM, got: {result!r}")


@expect_pass
def test_setrpm_above_max(ctx: TestContext) -> None:
    """
    'SETRPM 400' -> '!' (above valid range).
    """
    ctx.send_cmd("SETRPM 400")
    result = ctx.read_cmd_result(timeout_s=3.0)
    if result != "!":
        raise TestFailure(f"Expected '!' for above-range RPM, got: {result!r}")


@expect_pass
def test_setrpm_valid(ctx: TestContext) -> None:
    """
    'SETRPM 150' -> 'OK' (valid RPM).
    """
    ctx.send_cmd("SETRPM 150")
    result = ctx.read_cmd_result(timeout_s=3.0)
    if result != "OK":
        raise TestFailure(f"Expected 'OK' for valid RPM, got: {result!r}")


@expect_pass
def test_setrpm_garbage(ctx: TestContext) -> None:
    """
    'SETRPM qwertyuiopasdfghjklzxcvbnm1234567890' -> '!' (invalid token).
    """
    ctx.send_cmd("SETRPM qwertyuiopasdfghjklzxcvbnm1234567890")
    result = ctx.read_cmd_result(timeout_s=3.0)
    if result != "!":
        raise TestFailure(f"Expected '!' for invalid argument, got: {result!r}")

@expect_pass
def test_setrpm_persists_across_reboot(ctx: TestContext) -> None:
    """
    Set RPM to 150, reboot the device, and verify SHOW reports Setpoint RPM: 150.
    """
    # 1) Set RPM and confirm immediate success
    ctx.send_cmd("SETRPM 150")
    token = ctx.read_cmd_result(timeout_s=3.0)
    if token != "OK":
        raise TestFailure(f"Expected 'OK' after SETRPM 150, got: {token!r}")

    # 2) Disconnect, prompt user to reboot, then reconnect
    try:
        ctx.serial.close()
    except Exception:
        pass  # If already closed, ignore
    ctx.prompt("Please power-cycle the unit now (OFF → ON).", require_yes=False)
    ctx.sleep(2.0)  # brief settle
    ctx.reconnect()

    # 3) Issue SHOW and capture the multi-line response until terminal 'OK'
    ctx.send_cmd("SHOW")

    lines: List[str] = []
    deadline = time.monotonic() + 4.0  # overall timeout for the report
    while time.monotonic() < deadline:
        line = ctx.read_line(timeout_s=0.6)
        if line is None:
            continue
        if line == "OK":
            break
        lines.append(line)
    else:
        raise TestFailure("Timed out waiting for 'OK' after SHOW.")

    # 4) Parse the Setpoint line and verify it equals 150
    setpoint_line = next((ln for ln in lines if "Setpoint RPM:" in ln), None)
    if setpoint_line is None:
        raise TestFailure("SHOW output missing 'Setpoint RPM:' line.")

    # Allow flexible spacing; extract the first integer on that line
    import re
    m = re.search(r"Setpoint\s+RPM:\s*(\d+)", setpoint_line)
    if not m:
        raise TestFailure(f"Could not parse Setpoint RPM from: {setpoint_line!r}")

    setpoint = int(m.group(1))
    if setpoint != 150:
        raise TestFailure(f"Expected Setpoint RPM 150, got {setpoint}.")


# ---- SETKP ----
@expect_pass
def test_setkp_no_args(ctx: TestContext) -> None:
    ctx.send_cmd("SETKP")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETKP with no args, got {tok!r}")

@expect_pass
def test_setkp_two_args(ctx: TestContext) -> None:
    ctx.send_cmd("SETKP 1 2")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETKP with two args, got {tok!r}")

@expect_pass
def test_setkp_below_range(ctx: TestContext) -> None:
    ctx.send_cmd("SETKP -1")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETKP below range, got {tok!r}")

@expect_pass
def test_setkp_above_range(ctx: TestContext) -> None:
    ctx.send_cmd("SETKP 101")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETKP above range, got {tok!r}")

@expect_pass
def test_setkp_junk(ctx: TestContext) -> None:
    ctx.send_cmd("SETKP ???!!!@@@###")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETKP junk input, got {tok!r}")

@expect_pass
def test_setkp_valid_persists(ctx: TestContext) -> None:
    ctx.send_cmd("SETKP 0.05")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "OK":
        raise TestFailure(f"Expected 'OK' after SETKP 0.05, got {tok!r}")

    try:
        ctx.serial.close()
    except Exception:
        pass
    ctx.prompt("Power-cycle the unit now (OFF → ON). Press Enter when back on.")
    ctx.sleep(2.0)
    ctx.reconnect()

    lines = _read_show_until_ok(ctx)
    kp = _parse_pid_value(lines, "Kp")
    if abs(kp - 0.05) > 1e-6:
        raise TestFailure(f"Expected Kp 0.05, got {kp}.")

# ---- SETKI ----
@expect_pass
def test_setki_no_args(ctx: TestContext) -> None:
    ctx.send_cmd("SETKI")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETKI with no args, got {tok!r}")

@expect_pass
def test_setki_two_args(ctx: TestContext) -> None:
    ctx.send_cmd("SETKI 1 2")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETKI with two args, got {tok!r}")

@expect_pass
def test_setki_below_range(ctx: TestContext) -> None:
    ctx.send_cmd("SETKI -1")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETKI below range, got {tok!r}")

@expect_pass
def test_setki_above_range(ctx: TestContext) -> None:
    ctx.send_cmd("SETKI 101")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETKI above range, got {tok!r}")

@expect_pass
def test_setki_junk(ctx: TestContext) -> None:
    ctx.send_cmd("SETKI ???!!!@@@###")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETKI junk input, got {tok!r}")

@expect_pass
def test_setki_valid_persists(ctx: TestContext) -> None:
    ctx.send_cmd("SETKI 0.05")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "OK":
        raise TestFailure(f"Expected 'OK' after SETKI 0.05, got {tok!r}")

    try:
        ctx.serial.close()
    except Exception:
        pass
    ctx.prompt("Power-cycle the unit now (OFF → ON). Press Enter when back on.")
    ctx.sleep(2.0)
    ctx.reconnect()

    lines = _read_show_until_ok(ctx)
    ki = _parse_pid_value(lines, "Ki")
    if abs(ki - 0.05) > 1e-6:
        raise TestFailure(f"Expected Ki 0.05, got {ki}.")

# ---- SETKD ----
@expect_pass
def test_setkd_no_args(ctx: TestContext) -> None:
    ctx.send_cmd("SETKD")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETKD with no args, got {tok!r}")

@expect_pass
def test_setkd_two_args(ctx: TestContext) -> None:
    ctx.send_cmd("SETKD 1 2")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETKD with two args, got {tok!r}")

@expect_pass
def test_setkd_below_range(ctx: TestContext) -> None:
    ctx.send_cmd("SETKD -1")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETKD below range, got {tok!r}")

@expect_pass
def test_setkd_above_range(ctx: TestContext) -> None:
    ctx.send_cmd("SETKD 101")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETKD above range, got {tok!r}")

@expect_pass
def test_setkd_junk(ctx: TestContext) -> None:
    ctx.send_cmd("SETKD ???!!!@@@###")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETKD junk input, got {tok!r}")

@expect_pass
def test_setkd_valid_persists(ctx: TestContext) -> None:
    ctx.send_cmd("SETKD 0.05")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "OK":
        raise TestFailure(f"Expected 'OK' after SETKD 0.05, got {tok!r}")

    try:
        ctx.serial.close()
    except Exception:
        pass
    ctx.prompt("Power-cycle the unit now (OFF → ON). Press Enter when back on.")
    ctx.sleep(2.0)
    ctx.reconnect()

    lines = _read_show_until_ok(ctx)
    kd = _parse_pid_value(lines, "Kd")
    if abs(kd - 0.05) > 1e-6:
        raise TestFailure(f"Expected Kd 0.05, got {kd}.")

# ---- SETCURRENTLIM ----
@expect_pass
def test_setcurrentlim_no_args(ctx: TestContext) -> None:
    ctx.send_cmd("SETCURRENTLIM")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(
            f"Expected '!' for SETCURRENTLIM with no args, got {tok!r}"
        )


@expect_pass
def test_setcurrentlim_two_args(ctx: TestContext) -> None:
    ctx.send_cmd("SETCURRENTLIM 200 300")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(
            f"Expected '!' for SETCURRENTLIM with two args, got {tok!r}"
        )


@expect_pass
def test_setcurrentlim_below_range(ctx: TestContext) -> None:
    ctx.send_cmd("SETCURRENTLIM 99")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(
            f"Expected '!' for SETCURRENTLIM below range, got {tok!r}"
        )


@expect_pass
def test_setcurrentlim_above_range(ctx: TestContext) -> None:
    ctx.send_cmd("SETCURRENTLIM 1001")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(
            f"Expected '!' for SETCURRENTLIM above range, got {tok!r}"
        )


@expect_pass
def test_setcurrentlim_junk(ctx: TestContext) -> None:
    ctx.send_cmd("SETCURRENTLIM ???!!!@@@###")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(
            f"Expected '!' for SETCURRENTLIM junk input, got {tok!r}"
        )


@expect_pass
def test_setcurrentlim_valid_persists(ctx: TestContext) -> None:
    """
    Set a valid current limit (e.g., 200 mA), verify 'OK', power-cycle,
    then confirm SHOW reports that same value.
    """
    target_ma = 200  # choose any valid value 100..1000
    ctx.send_cmd(f"SETCURRENTLIM {target_ma}")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "OK":
        raise TestFailure(
            f"Expected 'OK' after SETCURRENTLIM {target_ma}, got {tok!r}"
        )

    # Power-cycle and reconnect
    try:
        ctx.serial.close()
    except Exception:
        pass
    ctx.prompt("Power-cycle the unit now (OFF → ON). Press Enter when back on.")
    ctx.sleep(2.0)
    ctx.reconnect()

    # Verify via SHOW
    lines = _read_show_until_ok(ctx, timeout_s=5.0)
    limit_ma = _parse_current_limit_ma(lines)
    if limit_ma != target_ma:
        raise TestFailure(
            f"Expected Current Limit {target_ma} mA, got {limit_ma} mA."
        )

# ---- SETCUTOFF ----
@expect_pass
def test_setcutoff_no_args(ctx: TestContext) -> None:
    ctx.send_cmd("SETCUTOFF")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETCUTOFF with no args, got {tok!r}")


@expect_pass
def test_setcutoff_two_args(ctx: TestContext) -> None:
    ctx.send_cmd("SETCUTOFF 0 1")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETCUTOFF with two args, got {tok!r}")


@expect_pass
def test_setcutoff_invalid_positive(ctx: TestContext) -> None:
    ctx.send_cmd("SETCUTOFF 2")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETCUTOFF with value 2, got {tok!r}")


@expect_pass
def test_setcutoff_invalid_negative(ctx: TestContext) -> None:
    ctx.send_cmd("SETCUTOFF -1")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETCUTOFF with value -1, got {tok!r}")


@expect_pass
def test_setcutoff_non_numeric(ctx: TestContext) -> None:
    ctx.send_cmd("SETCUTOFF abcdef")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETCUTOFF non-numeric input, got {tok!r}")

@expect_pass
def test_setcutoff_disable_persists(ctx: TestContext) -> None:
    """
    SETCUTOFF 0 -> OK, power-cycle, SHOW should report DISABLED.
    """
    ctx.send_cmd("SETCUTOFF 0")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "OK":
        raise TestFailure(f"Expected 'OK' after SETCUTOFF 0, got {tok!r}")

    # Power-cycle and reconnect
    try:
        ctx.serial.close()
    except Exception:
        pass
    ctx.prompt("Power-cycle the unit (OFF → ON). Press Enter when back on.")
    ctx.sleep(2.0)
    ctx.reconnect()

    # Verify via SHOW
    lines = _read_show_until_ok(ctx, timeout_s=5.0)
    enabled = _parse_cutoff_enabled(lines)
    if enabled:
        raise TestFailure("Expected Current Cutoff DISABLED after SETCUTOFF 0.")


@expect_pass
def test_setcutoff_enable_persists(ctx: TestContext) -> None:
    """
    SETCUTOFF 1 -> OK, power-cycle, SHOW should report ENABLED.
    """
    ctx.send_cmd("SETCUTOFF 1")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "OK":
        raise TestFailure(f"Expected 'OK' after SETCUTOFF 1, got {tok!r}")

    # Power-cycle and reconnect
    try:
        ctx.serial.close()
    except Exception:
        pass
    ctx.prompt("Power-cycle the unit (OFF → ON). Press Enter when back on.")
    ctx.sleep(2.0)
    ctx.reconnect()

    # Verify via SHOW
    lines = _read_show_until_ok(ctx, timeout_s=5.0)
    enabled = _parse_cutoff_enabled(lines)
    if not enabled:
        raise TestFailure("Expected Current Cutoff ENABLED after SETCUTOFF 1.")

# ---- SETRESTART ----
@expect_pass
def test_setrestart_no_args(ctx: TestContext) -> None:
    ctx.send_cmd("SETRESTART")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETRESTART with no args, got {tok!r}")


@expect_pass
def test_setrestart_two_args(ctx: TestContext) -> None:
    ctx.send_cmd("SETRESTART 0 1")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETRESTART with two args, got {tok!r}")


@expect_pass
def test_setrestart_invalid_positive(ctx: TestContext) -> None:
    ctx.send_cmd("SETRESTART 2")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETRESTART with value 2, got {tok!r}")


@expect_pass
def test_setrestart_invalid_negative(ctx: TestContext) -> None:
    ctx.send_cmd("SETRESTART -1")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(f"Expected '!' for SETRESTART with value -1, got {tok!r}")


@expect_pass
def test_setrestart_non_numeric(ctx: TestContext) -> None:
    ctx.send_cmd("SETRESTART abcdef")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "!":
        raise TestFailure(
            f"Expected '!' for SETRESTART non-numeric input, got {tok!r}"
        )

@expect_pass
def test_setrestart_disable_persists(ctx: TestContext) -> None:
    """
    SETRESTART 0 -> OK, power-cycle, SHOW should report Auto Restart: DISABLED.
    """
    ctx.send_cmd("SETRESTART 0")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "OK":
        raise TestFailure(f"Expected 'OK' after SETRESTART 0, got {tok!r}")

    # Power-cycle and reconnect
    try:
        ctx.serial.close()
    except Exception:
        pass
    ctx.prompt("Power-cycle the unit (OFF → ON). Press Enter when back on.")
    ctx.sleep(2.0)
    ctx.reconnect()

    lines = _read_show_until_ok(ctx, timeout_s=5.0)
    enabled = _parse_auto_restart_enabled(lines)
    if enabled:
        raise TestFailure("Expected Auto Restart DISABLED after SETRESTART 0.")


@expect_pass
def test_setrestart_enable_persists(ctx: TestContext) -> None:
    """
    SETRESTART 1 -> OK, power-cycle, SHOW should report Auto Restart: ENABLED.
    """
    ctx.send_cmd("SETRESTART 1")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "OK":
        raise TestFailure(f"Expected 'OK' after SETRESTART 1, got {tok!r}")

    # Power-cycle and reconnect
    try:
        ctx.serial.close()
    except Exception:
        pass
    ctx.prompt("Power-cycle the unit (OFF → ON). Press Enter when back on.")
    ctx.sleep(2.0)
    ctx.reconnect()

    lines = _read_show_until_ok(ctx, timeout_s=5.0)
    enabled = _parse_auto_restart_enabled(lines)
    if not enabled:
        raise TestFailure("Expected Auto Restart ENABLED after SETRESTART 1.")

# ---- SHOW ----
@expect_pass
def test_show_command_basic(ctx: TestContext) -> None:
    """
    Verify SHOW command responds with a settings block and terminates with 'OK'.
    """
    ctx.send_cmd("SHOW")

    lines: List[str] = []
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        line = ctx.read_line(timeout_s=0.6)
        if line is None:
            continue
        if line == "OK":
            break
        lines.append(line)
    else:
        raise TestFailure("Timed out waiting for 'OK' after SHOW.")

    if not lines:
        raise TestFailure("SHOW returned no content before 'OK'.")

    header = lines[0].strip()
    if not header.startswith("--- Motor Controller Settings"):
        raise TestFailure(
            f"SHOW output missing expected header, got: {header!r}"
        )

@expect_pass
def test_show_command_sanity(ctx: TestContext) -> None:
    """
    Verify SHOW output includes all expected labels and terminates with 'OK'.
    """
    expected_labels = [
        "Setpoint RPM:",
        "Kp:",
        "Ki:",
        "Kd:",
        "Current Cutoff:",
        "Current Limit:",
        "Auto Restart:",
        "Power Cycles:",
        "Log Head Index:",
    ]

    ctx.send_cmd("SHOW")

    lines: List[str] = []
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        line = ctx.read_line(timeout_s=0.6)
        if line is None:
            continue
        if line == "OK":
            break
        lines.append(line)
    else:
        raise TestFailure("Timed out waiting for 'OK' after SHOW.")

    text = "\n".join(lines)

    missing = [label for label in expected_labels if label not in text]
    if missing:
        raise TestFailure(f"SHOW output missing expected labels: {missing}")

# ---- DUMPLOG ----
@expect_pass
def test_dumplog_basic(ctx: TestContext) -> None:
    """
    Verify DUMPLOG returns multiple lines of log data and terminates with 'OK'.
    """
    ctx.send_cmd("DUMPLOG")

    lines: List[str] = []
    deadline = time.monotonic() + 15.0  # allow longer for large logs
    while time.monotonic() < deadline:
        line = ctx.read_line(timeout_s=1.0)
        if line is None:
            continue
        if line == "OK":
            break
        lines.append(line)
    else:
        raise TestFailure("Timed out waiting for 'OK' after DUMPLOG.")

    if not lines:
        raise TestFailure("DUMPLOG returned no log lines before 'OK'.")

    # Basic sanity: check that log lines look comma-separated
    sample = lines[0]
    if "," not in sample:
        raise TestFailure(f"First log line not in expected CSV format: {sample!r}")

# ---- RESETCONFIG ----
# ---- Power Cycle Count Verification Also Verified via Test----
@expect_pass
def test_resetconfig_restores_defaults(ctx: TestContext) -> None:
    """
    RESETCONFIG (no args) should restore factory defaults.
    Then after a reboot, defaults should persist and Power Cycles should increment.
    """
    # 1) Issue RESETCONFIG and require OK
    ctx.send_cmd("RESETCONFIG")
    tok = ctx.read_cmd_result(timeout_s=5.0)
    if tok != "OK":
        raise TestFailure(f"Expected 'OK' after RESETCONFIG, got {tok!r}")

    # 2) SHOW and verify defaults immediately after reset
    lines = _read_show_until_ok(ctx, timeout_s=5.0)

    sp = _parse_setpoint_rpm(lines)
    if sp != 210:
        raise TestFailure(f"Expected Setpoint RPM 210, got {sp}.")

    kp = _parse_pid_value(lines, "Kp")
    ki = _parse_pid_value(lines, "Ki")
    kd = _parse_pid_value(lines, "Kd")
    if abs(kp - 0.02) > 1e-6:
        raise TestFailure(f"Expected Kp 0.02, got {kp}.")
    if abs(ki - 0.05) > 1e-6:
        raise TestFailure(f"Expected Ki 0.05, got {ki}.")
    if abs(kd - 0.00) > 1e-6:
        raise TestFailure(f"Expected Kd 0.00, got {kd}.")

    cutoff_enabled = _parse_cutoff_enabled(lines)
    if not cutoff_enabled:
        raise TestFailure("Expected Current Cutoff ENABLED after RESETCONFIG.")

    autorestart_enabled = _parse_auto_restart_enabled(lines)
    if not autorestart_enabled:
        raise TestFailure("Expected Auto Restart ENABLED after RESETCONFIG.")

    limit_ma = _parse_current_limit_ma(lines)
    if limit_ma != 300:
        raise TestFailure(f"Expected Current Limit 300 mA, got {limit_ma} mA.")

    power_cycles_1 = _parse_label_int(lines, "Power Cycles")
    if power_cycles_1 != 1:
        raise TestFailure(f"Expected Power Cycles 1 after RESETCONFIG, got {power_cycles_1}.")

    log_head = _parse_label_int(lines, "Log Head Index")
    if log_head != 0:
        raise TestFailure(f"Expected Log Head Index 0, got {log_head}.")

    # 3) Reboot, reconnect, and verify persistence; Power Cycles must increment to 2
    try:
        ctx.serial.close()
    except Exception:
        pass
    ctx.prompt("Power-cycle the unit (OFF → ON). Press Enter when back on.")
    ctx.sleep(2.0)
    ctx.reconnect()

    lines2 = _read_show_until_ok(ctx, timeout_s=5.0)

    # Defaults should be identical post-reboot
    sp2 = _parse_setpoint_rpm(lines2)
    if sp2 != 210:
        raise TestFailure(f"[Reboot] Expected Setpoint RPM 210, got {sp2}.")

    kp2 = _parse_pid_value(lines2, "Kp")
    ki2 = _parse_pid_value(lines2, "Ki")
    kd2 = _parse_pid_value(lines2, "Kd")
    if abs(kp2 - 0.02) > 1e-6:
        raise TestFailure(f"[Reboot] Expected Kp 0.02, got {kp2}.")
    if abs(ki2 - 0.05) > 1e-6:
        raise TestFailure(f"[Reboot] Expected Ki 0.05, got {ki2}.")
    if abs(kd2 - 0.00) > 1e-6:
        raise TestFailure(f"[Reboot] Expected Kd 0.00, got {kd2}.")

    cutoff_enabled2 = _parse_cutoff_enabled(lines2)
    if not cutoff_enabled2:
        raise TestFailure("[Reboot] Expected Current Cutoff ENABLED.")

    autorestart_enabled2 = _parse_auto_restart_enabled(lines2)
    if not autorestart_enabled2:
        raise TestFailure("[Reboot] Expected Auto Restart ENABLED.")

    limit_ma2 = _parse_current_limit_ma(lines2)
    if limit_ma2 != 300:
        raise TestFailure(f"[Reboot] Expected Current Limit 300 mA, got {limit_ma2} mA.")

    power_cycles_2 = _parse_label_int(lines2, "Power Cycles")
    if power_cycles_2 != 2:
        raise TestFailure(
            f"[Reboot] Expected Power Cycles to increment to 2, got {power_cycles_2}."
        )

    log_head2 = _parse_label_int(lines2, "Log Head Index")
    if log_head2 != 0:
        raise TestFailure(f"[Reboot] Expected Log Head Index 0, got {log_head2}.")

# ---- HELP ----
@expect_pass
def test_help_command_basic(ctx: TestContext) -> None:
    """
    Verify HELP command returns text content and terminates with 'OK'.
    """
    ctx.send_cmd("HELP")

    lines: List[str] = []
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        line = ctx.read_line(timeout_s=0.6)
        if line is None:
            continue
        if line == "OK":
            break
        lines.append(line)
    else:
        raise TestFailure("Timed out waiting for 'OK' after HELP.")

    if not lines:
        raise TestFailure("HELP returned no content before 'OK'.")

    # Optional sanity: check at least one line mentions a command
    if not any("SET" in ln or "SHOW" in ln for ln in lines):
        raise TestFailure("HELP output did not appear to list any commands.")

# ---- Junk Input Stress Test ----
@expect_pass
def test_junk_input_does_not_confuse_parser(ctx: TestContext) -> None:
    """
    Send a long invalid command string and verify that neither 'OK' nor '!' is returned.
    Then send a valid SHOW command and verify it still works.
    """
    # 1) Send junk
    junk_cmd = "THISISNOTACOMMAND12345"
    ctx.send_cmd(junk_cmd)
    
    # Read a few lines for a short window and make sure no OK/! tokens appear
    deadline = time.monotonic() + 3.0
    seen_result = False
    while time.monotonic() < deadline:
        line = ctx.read_line(timeout_s=0.5)
        if line is None:
            continue
        if line in {"OK", "!"}:
            seen_result = True
            break
    if seen_result:
        raise TestFailure("Unexpected result token ('OK' or '!') returned for junk input.")

    # 2) Issue a valid command (SHOW) and verify it terminates with OK
    ctx.send_cmd("SHOW")

    lines: List[str] = []
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        line = ctx.read_line(timeout_s=0.6)
        if line is None:
            continue
        if line == "OK":
            break
        lines.append(line)
    else:
        raise TestFailure("Timed out waiting for 'OK' after SHOW.")

    if not lines:
        raise TestFailure("SHOW returned no content after junk input.")

@expect_pass
def test_junk_very_long_then_show_ok(ctx: TestContext) -> None:
    """
    Send a very long invalid command (several hundred chars) and verify no 'OK'/'!' is returned.
    Then send SHOW and verify it still returns content and terminates with 'OK'.
    """
    junk = "X" * 800  # several hundred characters
    ctx.send_cmd(junk)

    # Listen briefly; ensure no terminal tokens are produced for junk.
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        line = ctx.read_line(timeout_s=0.5)
        if line is None:
            continue
        if line in {"OK", "!"}:
            raise TestFailure("Unexpected 'OK'/'!' returned for very long junk input.")

    # Now prove parser still works with a valid command.
    ctx.send_cmd("SHOW")

    lines: List[str] = []
    deadline = time.monotonic() + 8.0  # give SHOW time
    while time.monotonic() < deadline:
        line = ctx.read_line(timeout_s=0.6)
        if line is None:
            continue
        if line == "OK":
            break
        lines.append(line)
    else:
        raise TestFailure("Timed out waiting for 'OK' after SHOW following long junk.")

    if not lines:
        raise TestFailure("SHOW returned no content after long junk input.")

@expect_pass
def test_partial_command_with_terminator_no_response_then_show_ok(ctx: TestContext) -> None:
    """
    Send a partial valid-looking command ('SETCUR') terminated with CR/LF.
    Device should produce no response at all (no 'OK' or '!').
    Then send SHOW and verify normal operation.
    """
    # Send partial command with terminator
    ctx.send_cmd("SETCUR")

    # Observe for a short window; no 'OK' or '!' should appear
    deadline = time.monotonic() + 3.0
    seen_result = False
    while time.monotonic() < deadline:
        line = ctx.read_line(timeout_s=0.5)
        if line is None:
            continue
        if line in {"OK", "!"}:
            seen_result = True
            break
    if seen_result:
        raise TestFailure("Unexpected result token ('OK' or '!') returned for partial command.")

    # Now send a valid command to ensure parser still works
    ctx.send_cmd("SHOW")

    lines: List[str] = []
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        line = ctx.read_line(timeout_s=0.6)
        if line is None:
            continue
        if line == "OK":
            break
        lines.append(line)
    else:
        raise TestFailure("Timed out waiting for 'OK' after SHOW following partial command.")

    if not lines:
        raise TestFailure("SHOW returned no content after partial command test.")

# ---- Overcurrent Functionality ----
@expect_pass
def test_overcurrent_cutoff_no_autorestart_manual(ctx: TestContext) -> None:
    """
    Manual test:
      1) Ensure motor is running (SETRPM 210).
      2) Enable current cutoff (SETCUTOFF 1).
      3) Disable auto restart (SETRESTART 0).
      4) Set current limit to 100 mA (SETCURRENTLIM 100).
      5) Prompt operator to stall the motor. Expect cutoff to stop motor.
      6) Wait ~2 minutes. Motor should NOT auto-restart.
      7) Operator confirms pass/fail with 'y' or 'n'.
      8) Operator power-cycles the system; test reconnects before exit.
    """
    # Ensure motor target RPM
    ctx.send_cmd("SETRPM 210")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "OK":
        raise TestFailure(f"Expected 'OK' after SETRPM 210, got {tok!r}")
    ctx.sleep(2.0)  # brief spin-up

    # Enable cutoff
    ctx.send_cmd("SETCUTOFF 1")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "OK":
        raise TestFailure(f"Expected 'OK' after SETCUTOFF 1, got {tok!r}")

    # Disable auto-restart
    ctx.send_cmd("SETRESTART 0")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "OK":
        raise TestFailure(f"Expected 'OK' after SETRESTART 0, got {tok!r}")

    # Set strict current limit
    ctx.send_cmd("SETCURRENTLIM 250")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "OK":
        raise TestFailure(f"Expected 'OK' after SETCURRENTLIM 250, got {tok!r}")

    # Operator instructions
    print(
        "Manual step:\n"
        "  • Gently STALL the motor (mechanically load it until it stops).\n"
        "  • The controller should CUT OFF the motor due to overcurrent.\n"
        "  • Wait TWO MINUTES. The controller should NOT auto-restart the motor.\n"
        "Observe this sequence now..."
    )

    # Allow time for cutoff and potential restart (should NOT happen)
    ctx.sleep(120.0)

    # Operator confirmation
    ans = input("Did the motor cut off on stall AND remain off after 2 minutes (no auto-restart)? [y/n]: ").strip().lower()
    if ans != "y":
        raise TestFailure(
            "Operator reported that cutoff did not behave as expected (either no cutoff or unexpected restart)."
        )

    # Prompt for reboot to restore clean state
    try:
        ctx.serial.close()
    except Exception:
        pass
    ctx.prompt("Please power-cycle the unit now (OFF → ON). Press Enter when back on.")
    ctx.sleep(2.0)
    ctx.reconnect()

# ---- Auto Restart Functionality ----
@expect_pass
def test_overcurrent_cutoff_and_autorestart_manual(ctx: TestContext) -> None:
    """
    Manual test:
      1) Ensure motor is running (SETRPM 210).
      2) Enable current cutoff (SETCUTOFF 1).
      3) Enable auto restart (SETRESTART 1).
      4) Set current limit to 100 mA (SETCURRENTLIM 100).
      5) Prompt operator to stall the motor. Expect cutoff to stop motor.
      6) After ~1 minute, expect motor to auto-restart.
      7) Operator confirms pass/fail with 'y' or 'n'.
    """
    # Make sure the motor has a target RPM so the stall is meaningful.
    ctx.send_cmd("SETRPM 210")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "OK":
        raise TestFailure(f"Expected 'OK' after SETRPM 210, got {tok!r}")
    ctx.sleep(2.0)  # brief spin-up

    # Enable cutoff
    ctx.send_cmd("SETCUTOFF 1")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "OK":
        raise TestFailure(f"Expected 'OK' after SETCUTOFF 1, got {tok!r}")

    # Enable auto-restart
    ctx.send_cmd("SETRESTART 1")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "OK":
        raise TestFailure(f"Expected 'OK' after SETRESTART 1, got {tok!r}")

    # Set strict current limit
    ctx.send_cmd("SETCURRENTLIM 250")
    tok = ctx.read_cmd_result(timeout_s=3.0)
    if tok != "OK":
        raise TestFailure(f"Expected 'OK' after SETCURRENTLIM 250, got {tok!r}")

    # Operator instructions
    print(
        "Manual step:\n"
        "  • Gently STALL the motor (mechanically load it until it stops).\n"
        "  • The controller should CUT OFF the motor due to overcurrent.\n"
        "  • After approximately ONE MINUTE, the controller should AUTO-RESTART the motor.\n"
        "Observe this sequence now..."
    )

    # Allow enough time for cutoff and restart
    ctx.sleep(70.0)

    # Operator confirmation
    ans = input("Did the motor cut off on stall AND then auto-restart after ~1 minute? [y/n]: ").strip().lower()
    if ans != "y":
        raise TestFailure(
            "Operator reported that cutoff and/or auto-restart did not behave as expected."
        )

# ---- RPM Verification ----
@expect_pass
def test_voltage_step_recovery_manual(ctx: TestContext) -> None:
    """
    Manual test:
      1) RESETCONFIG -> defaults (Setpoint RPM 210).
      2) Confirm the motor is running at target (allow brief spin-up).
      3) Operator drops drive voltage by 2 V (e.g., 9 V -> 7 V); verify RPM recovers within 60 s.
      4) Operator restores drive voltage to 9 V; verify RPM recovers within 60 s.
      5) Operator confirms each recovery with 'y' or 'n'.
    """
    # Reset to defaults
    ctx.send_cmd("RESETCONFIG")
    tok = ctx.read_cmd_result(timeout_s=5.0)
    if tok != "OK":
        raise TestFailure(f"Expected 'OK' after RESETCONFIG, got {tok!r}")

    # Optional: quick SHOW to ensure the setpoint is 210 (silent sanity)
    try:
        lines = _read_show_until_ok(ctx, timeout_s=5.0)
        sp = _parse_setpoint_rpm(lines)
        if sp != 210:
            raise TestFailure(f"Expected Setpoint RPM 210 after reset, got {sp}.")
    except Exception as exc:
        # If SHOW is noisy during boot, give it a moment and retry once
        ctx.sleep(1.0)
        lines = _read_show_until_ok(ctx, timeout_s=5.0)
        sp = _parse_setpoint_rpm(lines)
        if sp != 210:
            raise TestFailure(f"Expected Setpoint RPM 210 after reset, got {sp}.") from exc

    # Allow time for motor to spin up at the default setpoint
    ctx.sleep(2.0)

    # Step 1: Decrease drive voltage by 2 V (e.g., 9 V -> 7 V)
    print(
        "Manual step A:\n"
        "  • Reduce the DRIVE VOLTAGE by 2 V (e.g., from 9 V down to 7 V).\n"
        "  • The controller should recover the RPM back to the 210 RPM setpoint within 60 seconds."
    )
    input("Press Enter to start the 60-second observation window...")
    ctx.sleep(60.0)
    ans = input("Did RPM recover to ~210 RPM within 60 seconds after the voltage drop? [y/n]: ").strip().lower()
    if ans != "y":
        raise TestFailure("Operator reported RPM did not recover after voltage drop.")

    # Step 2: Restore drive voltage to original 9 V
    print(
        "Manual step B:\n"
        "  • Restore the DRIVE VOLTAGE to the original 9 V.\n"
        "  • The controller should again recover to ~210 RPM within 60 seconds."
    )
    input("Press Enter to start the next 60-second observation window...")
    ctx.sleep(60.0)
    ans2 = input("Did RPM recover to ~210 RPM within 60 seconds after restoring to 9 V? [y/n]: ").strip().lower()
    if ans2 != "y":
        raise TestFailure("Operator reported RPM did not recover after restoring voltage.")

# =========================
# CLI
# =========================

def main() -> int:
    parser = argparse.ArgumentParser(description="Serial hardware test runner.")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate.")
    parser.add_argument("--timeout", type=float, default=1.0, help="Serial timeout.")
    parser.add_argument("--pattern", type=str, default=None, help="Substring to filter tests (e.g., 'echo').")
    parser.add_argument("--list", action="store_true", help="List discovered tests and exit.")
    parser.add_argument("--post-reset", dest="post_reset", action="store_true", default=True,
                        help="Restore factory defaults after tests (default: on).")
    parser.add_argument("--no-post-reset", dest="post_reset", action="store_false",
                        help="Do not restore defaults after tests.")
    parser.add_argument("--post-reboot", action="store_true", default=False,
                        help="After post-reset, prompt to power-cycle and re-verify.")
    args = parser.parse_args()

    if args.list:
        print("Discovered tests:")
        for name, (_, exp) in sorted(_TEST_REGISTRY.items()):
            print(f"- {name} [{exp}]")
        return 0

    try:
        ser = choose_serial_port(baudrate=args.baud, timeout=args.timeout)
    except Exception as exc:
        print(f"Error opening serial port: {exc}", file=sys.stderr)
        return 2

    try:
        ctx = TestContext(ser)
        run_tests(ctx, pattern=args.pattern)
    finally:
        try:
            if "ctx" in locals() and args.post_reset:
                print("\nPost-run: restoring factory defaults...")
                reset_to_defaults(ctx, verify=True, reboot=args.post_reboot)
        except Exception as exc:
            print(f"Warning during post-reset: {exc!r}", file=sys.stderr)
        finally:
            try:
                ser.close()
            except Exception:
                pass

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
