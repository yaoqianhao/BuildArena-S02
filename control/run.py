"""Run the S02 rocket orbit-return example from the repository root.

Usage:
    uv run python control/examples/s02_rocket_orbit_return/run.py
    uv run python control/examples/s02_rocket_orbit_return/run.py --prepare-only
    uv run python control/examples/s02_rocket_orbit_return/run.py --edit-before-start
"""

import sys
from pathlib import Path

_root = Path(__file__).resolve()
while _root.parent != _root and not (_root / "pyproject.toml").exists():
    _root = _root.parent
REPO = _root
sys.path[:0] = [str(REPO), str(REPO / "control")]

from control.run_example import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main(default_example="s02_rocket_orbit_return"))
