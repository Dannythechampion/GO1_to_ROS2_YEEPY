import subprocess
import sys
from pathlib import Path


SCRIPT = Path(__file__).with_name("patch_livox_latest_frame_queue.py")


def test_patch_keeps_latest_frames_when_publish_queue_is_full(tmp_path):
    comm = tmp_path / "comm.cpp"
    lds = tmp_path / "lds.cpp"
    comm.write_text(
        "uint32_t CalculatePacketQueueSize(const double publish_freq) {\n"
        "  uint32_t queue_size = 10;\n"
        "  if (publish_freq > 10.0) {\n"
        "    queue_size = static_cast<uint32_t>(publish_freq) + 1;\n"
        "  }\n"
        "  return queue_size;\n"
        "}\n",
        encoding="utf-8",
    )
    lds.write_text(
        "  } else {\n"
        "    if (pcd_semaphore_.GetCount() <= 0) {\n"
        "        pcd_semaphore_.Signal();\n"
        "    }\n"
        "  }\n",
        encoding="utf-8",
    )
    result = subprocess.run(
        [sys.executable, str(SCRIPT), str(comm), str(lds)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "return 2;" in comm.read_text(encoding="utf-8")
    patched_lds = lds.read_text(encoding="utf-8")
    assert "QueuePop(queue, &discarded);" in patched_lds
    assert "QueuePushAny(queue, (uint8_t *)lidar_data, base_time);" in patched_lds
