#!/usr/bin/python3
"""Limit the shared launcher's pkill requests to its own Linux session."""
import os
from pathlib import Path
import re
import signal
import sys


def main():
    session = os.environ.get("DRONE3_SHARED_SESSION_ID", "")
    if not session.isdigit() or int(session) <= 1:
        print("shared pkill: managed session ID is required", file=sys.stderr)
        return 2
    if len(sys.argv) != 3 or sys.argv[1] not in ("-x", "-f"):
        print("shared pkill: unsupported arguments", file=sys.stderr)
        return 2
    mode, pattern = sys.argv[1:]
    if (mode, pattern) not in {
        ("-x", "cf2"), ("-f", "crazysim.py"), ("-f", "crazysim_cpx.py")
    }:
        print("shared pkill: request outside the baseline cleanup contract", file=sys.stderr)
        return 2
    hits = 0
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid == os.getpid():
            continue
        try:
            if os.getsid(pid) != int(session):
                continue
            if mode == "-x":
                matched = (entry / "comm").read_text().strip() == pattern
            else:
                command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
                matched = re.search(pattern, command) is not None
            if matched:
                os.kill(pid, signal.SIGTERM)
                hits += 1
        except (ProcessLookupError, PermissionError, FileNotFoundError):
            continue
    return 0 if hits else 1


if __name__ == "__main__":
    sys.exit(main())
