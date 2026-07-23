#ifndef GO1_MAPPING__PCD_PROJECTION_HPP_
#define GO1_MAPPING__PCD_PROJECTION_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <istream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace go1_mapping
{

// Match the session writer's 256 MiB retained PointXYZI payload budget.
inline constexpr std::size_t kMaximumMergedCloudBytes = 268435456U;
// Bound all modeled live buffers to 2 GiB on the Jetson, including a
// conservative six-payload VoxelGrid input/output/workspace allowance.
inline constexpr std::size_t kProcessMemoryBudgetBytes = 2147483648ULL;
inline constexpr std::size_t kVoxelFilterPayloadCopies = 6U;

enum class PcdDataEncoding
{
  Ascii,
  Binary,
  BinaryCompressed
};

enum class MemoryPhase
{
  LoadChunk,
  FilterChunk,
  AppendChunk,
  FilterMerged
};

struct AggregateMemoryState
{
  std::size_t merged_bytes;
  std::size_t incoming_bytes;
  std::size_t file_bytes;
  std::size_t combined_bytes;
  std::size_t raw_decoded_bytes;
};

struct PcdHeaderMetadata
{
  PcdDataEncoding encoding;
  std::size_t point_count;
  std::size_t raw_point_step;
  std::size_t raw_decoded_bytes;
  std::size_t decoded_bytes;
  std::size_t compressed_bytes;
  std::size_t data_offset;
  std::size_t file_bytes;
};

struct ProjectionBounds
{
  double min_z;
  double max_z;
};

struct Grid
{
  std::size_t width;
  std::size_t height;
  double resolution;
  double origin_x;
  double origin_y;
  std::vector<std::uint8_t> cells;
};

struct PublicationArtifact
{
  std::filesystem::path temporary;
  std::filesystem::path final;
};

struct PublicationOperations
{
  std::function<void(const std::filesystem::path &, const std::filesystem::path &)>
  link_no_replace;
  std::function<void(const std::filesystem::path &)> unlink_path;
  std::function<void(const std::filesystem::path &)> sync_directory;
};

ProjectionBounds z_bounds(double sensor_height_m);

std::size_t point_payload_bytes(std::size_t point_count);

std::size_t checked_merged_point_count(
  std::size_t existing_points,
  std::size_t incoming_points,
  std::size_t max_bytes = kMaximumMergedCloudBytes);

std::size_t estimated_peak_bytes(
  MemoryPhase phase,
  AggregateMemoryState state);

void enforce_memory_budget(
  MemoryPhase phase,
  AggregateMemoryState state,
  std::size_t budget_bytes = kProcessMemoryBudgetBytes);

PcdHeaderMetadata preflight_pcd_header(
  std::istream & input,
  std::size_t file_bytes,
  std::size_t retained_merged_bytes,
  std::size_t budget_bytes = kProcessMemoryBudgetBytes);


pcl::PointCloud<pcl::PointXYZI> voxel_filter_finite(
  pcl::PointCloud<pcl::PointXYZI> && cloud,
  double voxel_size,
  std::size_t retained_merged_bytes = 0U,
  std::size_t budget_bytes = kProcessMemoryBudgetBytes);

Grid project_occupied(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  double resolution,
  ProjectionBounds bounds);

void publish_artifacts(
  const std::vector<PublicationArtifact> & artifacts,
  const PublicationOperations & operations);

std::string yaml_quote(const std::string & value);

}  // namespace go1_mapping

#endif  // GO1_MAPPING__PCD_PROJECTION_HPP_
