#!/usr/bin/env python3
"""Bound Livox Driver2's internal raw packet backlog for real-time navigation."""

import sys
from pathlib import Path


OLD = "    self->raw_packet_queue_.push_back(packet);"
NEW = """    constexpr size_t kMaxRealtimePacketBacklog = 256;
    while (self->raw_packet_queue_.size() >= kMaxRealtimePacketBacklog) {
      self->raw_packet_queue_.pop_front();
    }
    self->raw_packet_queue_.push_back(packet);"""


def patch(source_path: Path) -> None:
    source = source_path.read_text(encoding="utf-8")
    if NEW in source:
        return
    if source.count(OLD) != 1:
        raise RuntimeError(f"expected exactly one Livox raw packet enqueue in {source_path}")
    source_path.write_text(source.replace(OLD, NEW), encoding="utf-8")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(f"usage: {Path(sys.argv[0]).name} PUB_HANDLER_CPP")
    patch(Path(sys.argv[1]))
