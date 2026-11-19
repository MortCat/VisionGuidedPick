"""Bound the stdout/stderr log of a single lock-owned ROS component."""
import sys
from pathlib import Path

MAX_BYTES = 32 * 1024 * 1024
BACKUPS = 3


def rotate(path):
    for index in range(BACKUPS, 0, -1):
        previous = path if index == 1 else path.with_name(f"{path.name}.{index - 1}")
        if previous.exists():
            previous.replace(path.with_name(f"{path.name}.{index}"))


def capture(path, source):
    if path.exists() and path.stat().st_size >= MAX_BYTES:
        rotate(path)
    chunk = source.read1(64 * 1024)
    while chunk:
        with path.open("ab", buffering=0) as log:
            size = log.tell()
            while chunk and size + len(chunk) <= MAX_BYTES:
                log.write(chunk)
                size += len(chunk)
                chunk = source.read1(64 * 1024)
        if chunk:
            rotate(path)


if __name__ == "__main__":
    path = Path(sys.argv[1])
    try:
        capture(path, sys.stdin.buffer)
    except OSError as error:
        print(f"Unable to write {path}: {error}", file=sys.stderr)
        # Drain the pipe so a disk failure does not terminate the ROS process.
        while sys.stdin.buffer.read1(64 * 1024):
            pass
