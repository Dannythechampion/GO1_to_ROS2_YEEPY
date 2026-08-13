#!/usr/bin/env python3
"""Make Livox Driver2 prefer recent frames over an old publish backlog."""

import sys
from pathlib import Path


OLD_SIZE = """uint32_t CalculatePacketQueueSize(const double publish_freq) {
  uint32_t queue_size = 10;
  if (publish_freq > 10.0) {
    queue_size = static_cast<uint32_t>(publish_freq) + 1;
  }
  return queue_size;
}"""
NEW_SIZE = """uint32_t CalculatePacketQueueSize(const double publish_freq) {
  (void)publish_freq;
  return 2;
}"""
OLD_FULL = """  } else {
    if (pcd_semaphore_.GetCount() <= 0) {
        pcd_semaphore_.Signal();
    }
  }"""
NEW_FULL = """  } else {
    StoragePacket discarded;
    QueuePop(queue, &discarded);
    QueuePushAny(queue, (uint8_t *)lidar_data, base_time);
    if (pcd_semaphore_.GetCount() <= 0) {
      pcd_semaphore_.Signal();
    }
  }"""


def replace_once(path: Path, old: str, new: str) -> None:
    text = path.read_text(encoding="utf-8")
    if new in text:
        return
    if text.count(old) != 1:
        raise RuntimeError(f"expected exactly one patch location in {path}")
    path.write_text(text.replace(old, new), encoding="utf-8")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(f"usage: {Path(sys.argv[0]).name} COMM_CPP LDS_CPP")
    replace_once(Path(sys.argv[1]), OLD_SIZE, NEW_SIZE)
    replace_once(Path(sys.argv[2]), OLD_FULL, NEW_FULL)
