#ifndef GO1_MAPPING__PCD_CHUNK_STORAGE_HPP_
#define GO1_MAPPING__PCD_CHUNK_STORAGE_HPP_

#include <cstddef>
#include <filesystem>
#include <functional>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "go1_mapping/pcd_chunk_buffer.hpp"

namespace go1_mapping
{

struct ValidatedCloudMessage
{
  std::size_t point_count;
  std::size_t payload_bytes;
};

/**
 * Validate a PointCloud2 before converting it to PCL.
 *
 * max_payload_bytes is the application bound for retained, uncompressed
 * pcl::PointXYZI payload. The already DDS-owned message and the temporary PCL
 * compression workspace are separate overhead. Serialized message data is
 * nevertheless required to fit the same configured bound before conversion.
 */
ValidatedCloudMessage validate_cloud_message(
  const sensor_msgs::msg::PointCloud2 & message,
  std::size_t max_payload_bytes);

class PcdChunkStorage
{
public:
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;
  using SaveFunction = std::function<int(const std::filesystem::path &, const Cloud &)>;
  using DirectorySyncFunction = std::function<void(int)>;

  explicit PcdChunkStorage(
    std::filesystem::path output_dir,
    SaveFunction save_function = SaveFunction{},
    DirectorySyncFunction directory_sync_function = DirectorySyncFunction{});
  ~PcdChunkStorage();

  PcdChunkStorage(const PcdChunkStorage &) = delete;
  PcdChunkStorage & operator=(const PcdChunkStorage &) = delete;
  PcdChunkStorage(PcdChunkStorage &&) = delete;
  PcdChunkStorage & operator=(PcdChunkStorage &&) = delete;

  bool flush(ChunkBuffer & buffer);

private:
  std::size_t next_free_index() const;

  std::filesystem::path output_dir_;
  SaveFunction save_function_;
  DirectorySyncFunction directory_sync_function_;
  int output_dir_fd_{-1};
};

}  // namespace go1_mapping

#endif  // GO1_MAPPING__PCD_CHUNK_STORAGE_HPP_
