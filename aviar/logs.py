"""
Reading recorded command logs into StepCmds.

The log is a whitespace-separated text file: one header line naming the
columns, then one data row per move. The CALLER names the columns it wants, so
this module hardcodes no vocabulary of its own -- each entry point owns its own
header.
"""

from pathlib import Path
from typing import Iterator

from .config import DEFAULT_DT, DEG_PER_RAD
from .protocol import StepCmd


def iter_stepcmds_from_log(
    log_path: str | Path,
    trans_col: str,
    rot_col: str,
    cath_col: str | None = None,
    default_dt: float = DEFAULT_DT,
    assume_units: str = "rad",  # "rad" if the rotation column is rad/s, else "deg"
) -> Iterator[StepCmd]:
    """
    Yield one StepCmd per data row, held for the row's "dt" column (or
    default_dt when the log has none).

    Blank lines and lines starting with "#" are skipped, as are rows with
    fewer fields than the header or with dt <= 0. Columns are located by name,
    so extra columns are harmless and column order may change.

    Omit cath_col for a single-instrument log; v4_mm_s is then 0.0.
    """
    path = Path(log_path)
    if not path.exists():
        raise FileNotFoundError(f"Log not found: {path}")

    wanted = [trans_col, rot_col] + ([cath_col] if cath_col else [])

    with path.open("r", encoding="utf-8") as f:
        header = None
        for line in f:
            line = line.strip()
            if line:
                header = line.split()
                break
        if header is None:
            return

        for name in wanted:
            if name not in header:
                raise ValueError(f"Missing column '{name}' in log header. Found: {header}")
        i_trans, i_rot = header.index(trans_col), header.index(rot_col)
        i_cath = header.index(cath_col) if cath_col else None
        # "dt" is optional: logs recorded before it existed fall back to default_dt
        i_dt = header.index("dt") if "dt" in header else None

        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < len(header):
                continue

            dt = default_dt if i_dt is None else float(parts[i_dt])
            if dt <= 0:
                continue

            w = float(parts[i_rot])
            if assume_units == "rad":
                w *= DEG_PER_RAD

            yield StepCmd(
                v2_mm_s=float(parts[i_trans]),
                w2_deg_s=w,
                v4_mm_s=0.0 if i_cath is None else float(parts[i_cath]),
                dt=dt,
            )
