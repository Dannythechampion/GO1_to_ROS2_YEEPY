#ifndef GO1_MAPPING__PCD_CHUNK_BUFFER_HPP_
#define GO1_MAPPING__PCD_CHUNK_BUFFER_HPP_

#include <cstddef>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace go1_mapping
{

class ChunkBuffer
{
public:
  ChunkBuffer(std::size_t max_frames, std::size_t max_bytes);

  bool should_flush_before(std::size_t incoming_points) const;
  void append(const pcl::PointCloud<pcl::PointXYZI> & cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr take();
  const pcl::PointCloud<pcl::PointXYZI> & peek() const noexcept;
  void clear() noexcept;

  std::size_t frame_count() const noexcept;
  std::size_t byte_count() const noexcept;

private:
  std::size_t point_bytes(std::size_t point_count) const;

  const std::size_t max_frames_;
  const std::size_t max_bytes_;
  std::size_t frame_count_{0};
  std::size_t byte_count_{0};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
};

}  // namespace go1_mapping

#endif  // GO1_MAPPING__PCD_CHUNK_BUFFER_HPP_
