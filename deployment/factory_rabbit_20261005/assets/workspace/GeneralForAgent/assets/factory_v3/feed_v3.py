"""Compatibility entry for the active factory feeder; no diagnostic writer."""
from pathlib import Path
import omni.usd
path = Path(omni.usd.get_context().get_stage().GetRootLayer().realPath).parent / "runtime/feed_v3.py"
exec(compile(path.read_text(), str(path), "exec"), globals())
