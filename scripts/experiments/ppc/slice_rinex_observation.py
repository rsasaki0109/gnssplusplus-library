#!/usr/bin/env python3
"""Write the first N epochs of a RINEX 3 observation file."""

from __future__ import annotations

import argparse
from pathlib import Path


def slice_observation(source: Path, destination: Path, epoch_count: int) -> None:
    if epoch_count < 1:
        raise ValueError("epoch_count must be positive")

    destination.parent.mkdir(parents=True, exist_ok=True)
    epochs_written = 0
    with source.open("r", encoding="ascii", errors="strict", newline="") as reader:
        with destination.open("w", encoding="ascii", newline="") as writer:
            for line in reader:
                if line.startswith(">"):
                    if epochs_written == epoch_count:
                        break
                    epochs_written += 1
                writer.write(line)

    if epochs_written < epoch_count:
        destination.unlink(missing_ok=True)
        raise ValueError(
            f"requested {epoch_count} epochs, but {source} contains only {epochs_written}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--epochs", type=int, required=True)
    args = parser.parse_args()
    slice_observation(args.source, args.destination, args.epochs)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
