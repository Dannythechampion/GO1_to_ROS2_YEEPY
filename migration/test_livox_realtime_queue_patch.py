import subprocess
import sys
from pathlib import Path


SCRIPT = Path(__file__).with_name("patch_livox_realtime_queue.py")


def test_patch_bounds_raw_packet_backlog(tmp_path):
    source = tmp_path / "pub_handler.cpp"
    source.write_text("    self->raw_packet_queue_.push_back(packet);\n", encoding="utf-8")
    result = subprocess.run(
        [sys.executable, str(SCRIPT), str(source)], capture_output=True, text=True
    )
    patched = source.read_text(encoding="utf-8")
    assert result.returncode == 0, result.stderr
    assert "kMaxRealtimePacketBacklog = 256" in patched
    assert patched.index("pop_front()") < patched.index("push_back(packet)")
