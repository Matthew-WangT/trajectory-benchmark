"""Text report + flat metrics dictionary (written as JSON for gating / regression)."""

from __future__ import annotations

import json
from pathlib import Path


class Report:
    def __init__(self, echo: bool = True):
        self.lines: list[str] = []
        self.metrics: dict[str, float] = {}
        self.skipped: dict[str, str] = {}
        self.echo = echo

    def add(self, text: str = "") -> None:
        for line in text.split("\n"):
            self.lines.append(line)
            if self.echo:
                print(line, flush=True)

    def section(self, title: str) -> None:
        self.add("")
        self.add("=" * 78)
        self.add(title)
        self.add("=" * 78)

    def prepend(self, lines: list[str]) -> None:
        """Insert report lines without echoing them during analyzer execution."""
        self.lines = lines + [""] + self.lines

    def metric(self, key: str, value) -> None:
        """Flat key such as 'joint.delay_xcorr_ms.Joint2_L'; non-finite values are stored as None."""
        v = float(value)
        self.metrics[key] = v if v == v and abs(v) != float("inf") else None

    def write(self, out_dir: Path, meta: dict, report_name: str = "report.txt") -> None:
        (out_dir / report_name).write_text("\n".join(self.lines) + "\n")
        payload = {**meta, "skipped": self.skipped, "metrics": self.metrics}
        (out_dir / "metrics.json").write_text(json.dumps(payload, indent=1, ensure_ascii=False) + "\n")
