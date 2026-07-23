#include "go1_mapping/pcd_chunk_buffer.hpp"

#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace go1_mapping
{

ChunkBuffer::ChunkBuffer(const std::size_t max_frames, const std::size_t max_bytes)
: max_frames_(max_frames),
  max_bytes_(max_bytes),
  cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>())
{
  if (max_frames_ == 0) {
    throw std::invalid_argument("max_frames must be greater than zero");
  }
  if (max_bytes_ == 0) {
    throw std::invalid_argument("max_bytes must be greater than zero");
  }
}

std::size_t ChunkBuffer::point_bytes(const std::size_t point_count) const
{
  if (point_count > std::numeric_limits<std::size_t>::max() / sizeof(pcl::PointXYZI)) {
    throw std::length_error("point cloud byte size overflows size_t");
  }
  return point_count * sizeof(pcl::PointXYZI);
}

bool ChunkBuffer::should_flush_before(const std::size_t incoming_points) const
{
  if (frame_count_ == 0) {
    return false;
  }

  if (frame_count_ >= max_frames_) {
    return true;
  }

  try {
    const auto incoming_bytes = point_bytes(incoming_points);
    return incoming_bytes > max_bytes_ - byte_count_;
  } catch (const std::length_error &) {
    return true;
  }
}

void ChunkBuffer::append(const pcl::PointCloud<pcl::PointXYZI> & cloud)
{
  const auto incoming_bytes = point_bytes(cloud.size());
  if (incoming_bytes > max_bytes_) {
    throw std::length_error("incoming frame exceeds max_bytes");
  }
  if (frame_count_ >= max_frames_) {
    throw std::length_error("append would exceed max_frames");
  }
  if (incoming_bytes > max_bytes_ - byte_count_) {
    throw std::length_error("append would exceed max_bytes");
  }

  *cloud_ += cloud;
  ++frame_count_;
  byte_count_ += incoming_bytes;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr ChunkBuffer::take()
{
  auto replacement = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  cloud_.swap(replacement);
  frame_count_ = 0;
  byte_count_ = 0;
  return replacement;
}

std::size_t ChunkBuffer::frame_count() const noexcept
{
  return frame_count_;
}

std::size_t ChunkBuffer::byte_count() const noexcept
{
  return byte_count_;
}

}  // namespace go1_mapping
