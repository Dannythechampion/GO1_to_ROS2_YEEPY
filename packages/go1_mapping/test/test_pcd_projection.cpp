#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "go1_mapping/pcd_projection.hpp"

namespace
{

using go1_mapping::Grid;
using go1_mapping::ProjectionBounds;
using PcdCloud = pcl::PointCloud<pcl::PointXYZI>;
using VoxelFilterSignature = PcdCloud (*)(
  PcdCloud &&, double, std::size_t, std::size_t);
static_assert(std::is_same_v<
  decltype(&go1_mapping::voxel_filter_finite), VoxelFilterSignature>);
pcl::PointXYZI point(const float x, const float y, const float z)
{
  pcl::PointXYZI result;
  result.x = x;
  result.y = y;
  result.z = z;
  result.intensity = 1.0F;
  return result;
}

TEST(PcdProjection, ComputesHeightRelativeZBounds)
{
  const ProjectionBounds bounds = go1_mapping::z_bounds(1.05);

  EXPECT_DOUBLE_EQ(bounds.min_z, -0.90);
  EXPECT_DOUBLE_EQ(bounds.max_z, 0.75);
}

TEST(PcdProjection, MarksOnlyObstacleCellsAndNeverInfersFreeSpace)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.0F, 0.0F, 0.0F));
  cloud.push_back(point(2.0F, 1.0F, 1.0F));

  const Grid grid = go1_mapping::project_occupied(
    cloud, 0.05, go1_mapping::z_bounds(1.05));

  EXPECT_EQ(grid.resolution, 0.05);
  EXPECT_NE(
    std::find(grid.cells.begin(), grid.cells.end(), std::uint8_t{0}),
    grid.cells.end());
  EXPECT_NE(
    std::find(grid.cells.begin(), grid.cells.end(), std::uint8_t{205}),
    grid.cells.end());
  EXPECT_EQ(
    std::count(grid.cells.begin(), grid.cells.end(), std::uint8_t{254}), 0);
}

TEST(PcdProjection, RejectsInvalidInputs)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.0F, 0.0F, 0.0F));

  EXPECT_THROW(go1_mapping::z_bounds(0.0), std::invalid_argument);
  EXPECT_THROW(
    go1_mapping::project_occupied(
      cloud, 0.0, ProjectionBounds{-1.0, 1.0}),
    std::invalid_argument);

  pcl::PointCloud<pcl::PointXYZI> invalid_cloud;
  invalid_cloud.push_back(point(
    std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F));
  EXPECT_THROW(
    go1_mapping::project_occupied(
      invalid_cloud, 0.05, ProjectionBounds{-1.0, 1.0}),
    std::invalid_argument);
}

TEST(PcdProjection, RemovesNonFinitePointsEvenWhenCloudClaimsDense)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.is_dense = true;
  cloud.push_back(point(1.0F, 2.0F, 3.0F));
  cloud.push_back(point(
    std::numeric_limits<float>::quiet_NaN(), 2.0F, 3.0F));
  cloud.push_back(point(
    1.0F, 2.0F, std::numeric_limits<float>::infinity()));

  const auto filtered = go1_mapping::voxel_filter_finite(std::move(cloud), 0.10);

  ASSERT_EQ(filtered.size(), 1U);
  EXPECT_TRUE(std::isfinite(filtered.front().x));
  EXPECT_TRUE(std::isfinite(filtered.front().y));
  EXPECT_TRUE(std::isfinite(filtered.front().z));
  EXPECT_TRUE(filtered.is_dense);
}

TEST(PcdProjection, RejectsMergeGrowthBeforeAllocation)
{
  const std::size_t two_points = 2U * sizeof(pcl::PointXYZI);

  EXPECT_EQ(go1_mapping::checked_merged_point_count(1U, 1U, two_points), 2U);
  EXPECT_THROW(
    go1_mapping::checked_merged_point_count(2U, 1U, two_points),
    std::length_error);
  EXPECT_THROW(
    go1_mapping::checked_merged_point_count(
      std::numeric_limits<std::size_t>::max(), 1U,
      std::numeric_limits<std::size_t>::max()),
    std::length_error);
}

std::string schema_header(const std::string & encoding, const std::size_t points = 1U)
{
  return
    "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
    "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n" +
    std::string("WIDTH ") + std::to_string(points) + "\nHEIGHT 1\nPOINTS " +
    std::to_string(points) + "\nDATA " + encoding + "\n";
}

void append_little_u32(std::string & output, const std::uint32_t value)
{
  for (std::size_t shift = 0; shift < 32U; shift += 8U) {
    output.push_back(static_cast<char>((value >> shift) & 0xffU));
  }
}
TEST(PcdProjection, PreflightRejectsOversizedPcdBeforeCloudAllocation)
{
  std::istringstream header(
    "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
    "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
    "WIDTH 100000000\nHEIGHT 1\nPOINTS 100000000\nDATA binary_compressed\n");

  EXPECT_THROW(
    go1_mapping::preflight_pcd_header(header, 512U, 0U, 1048576U),
    std::length_error);
}

TEST(PcdProjection, AcceptsValidPcdLayoutsBeforeLoad)
{
  std::string ascii = schema_header("ascii") + "0 0 0 1\n";
  std::istringstream ascii_stream(ascii);
  const auto ascii_metadata = go1_mapping::preflight_pcd_header(
    ascii_stream, ascii.size(), 0U);
  EXPECT_EQ(ascii_metadata.encoding, go1_mapping::PcdDataEncoding::Ascii);
  EXPECT_EQ(ascii_metadata.raw_point_step, 16U);
  EXPECT_EQ(ascii_metadata.raw_decoded_bytes, 16U);

  std::string binary = schema_header("binary") + std::string(16U, '\0');
  std::istringstream binary_stream(binary);
  const auto binary_metadata = go1_mapping::preflight_pcd_header(
    binary_stream, binary.size(), 0U);
  EXPECT_EQ(binary_metadata.encoding, go1_mapping::PcdDataEncoding::Binary);

  std::string compressed = schema_header("binary_compressed");
  append_little_u32(compressed, 4U);
  append_little_u32(compressed, 16U);
  compressed.append(4U, '\0');
  std::istringstream compressed_stream(compressed);
  const auto compressed_metadata = go1_mapping::preflight_pcd_header(
    compressed_stream, compressed.size(), 0U);
  EXPECT_EQ(
    compressed_metadata.encoding,
    go1_mapping::PcdDataEncoding::BinaryCompressed);
  EXPECT_EQ(compressed_metadata.compressed_bytes, 4U);
}

TEST(PcdProjection, RejectsInvalidPcdSchemaAndPayloadsBeforeLoad)
{
  std::string unsupported = schema_header("future_codec");
  std::istringstream unsupported_stream(unsupported);
  EXPECT_THROW(
    go1_mapping::preflight_pcd_header(
      unsupported_stream, unsupported.size(), 0U),
    std::invalid_argument);

  std::string incompatible = schema_header("ascii");
  const auto type_position = incompatible.find("TYPE F F F F");
  incompatible.replace(type_position, 12U, "TYPE U F F F");
  std::istringstream incompatible_stream(incompatible);
  EXPECT_THROW(
    go1_mapping::preflight_pcd_header(
      incompatible_stream, incompatible.size(), 0U),
    std::invalid_argument);

  const std::string overflow =
    "FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
    "COUNT 18446744073709551615 1 1 1\n"
    "WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n";
  std::istringstream overflow_stream(overflow);
  EXPECT_THROW(
    go1_mapping::preflight_pcd_header(overflow_stream, overflow.size(), 0U),
    std::length_error);

  std::string truncated_binary = schema_header("binary") + std::string(8U, '\0');
  std::istringstream truncated_binary_stream(truncated_binary);
  EXPECT_THROW(
    go1_mapping::preflight_pcd_header(
      truncated_binary_stream, truncated_binary.size(), 0U),
    std::invalid_argument);

  std::string truncated_compressed = schema_header("binary_compressed");
  append_little_u32(truncated_compressed, 4U);
  append_little_u32(truncated_compressed, 16U);
  truncated_compressed.append(2U, '\0');
  std::istringstream truncated_compressed_stream(truncated_compressed);
  EXPECT_THROW(
    go1_mapping::preflight_pcd_header(
      truncated_compressed_stream, truncated_compressed.size(), 0U),
    std::invalid_argument);

  std::string wrong_uncompressed = schema_header("binary_compressed");
  append_little_u32(wrong_uncompressed, 4U);
  append_little_u32(wrong_uncompressed, 32U);
  wrong_uncompressed.append(4U, '\0');
  std::istringstream wrong_uncompressed_stream(wrong_uncompressed);
  EXPECT_THROW(
    go1_mapping::preflight_pcd_header(
      wrong_uncompressed_stream, wrong_uncompressed.size(), 0U),
    std::invalid_argument);

  std::string compressed_bomb = schema_header("binary_compressed");
  append_little_u32(compressed_bomb, std::numeric_limits<std::uint32_t>::max());
  append_little_u32(compressed_bomb, std::numeric_limits<std::uint32_t>::max());
  std::istringstream compressed_bomb_stream(compressed_bomb);
  EXPECT_THROW(
    go1_mapping::preflight_pcd_header(
      compressed_bomb_stream, compressed_bomb.size(), 0U),
    std::length_error);
}

TEST(PcdProjection, AccountsForEveryConcurrentAllocationState)
{
  using go1_mapping::AggregateMemoryState;
  using go1_mapping::MemoryPhase;
  const AggregateMemoryState state{
    100U, 200U, 50U, 300U, 200U};

  EXPECT_EQ(go1_mapping::estimated_peak_bytes(MemoryPhase::LoadChunk, state), 550U);
  EXPECT_EQ(go1_mapping::estimated_peak_bytes(MemoryPhase::FilterChunk, state), 1300U);
  EXPECT_EQ(go1_mapping::estimated_peak_bytes(MemoryPhase::AppendChunk, state), 600U);
  EXPECT_EQ(go1_mapping::estimated_peak_bytes(MemoryPhase::FilterMerged, state), 600U);
  EXPECT_THROW(
    go1_mapping::estimated_peak_bytes(
      MemoryPhase::LoadChunk,
      AggregateMemoryState{
        std::numeric_limits<std::size_t>::max(), 1U, 0U, 0U, 0U}),
    std::length_error);

  for (const auto phase : {
      MemoryPhase::LoadChunk, MemoryPhase::FilterChunk,
      MemoryPhase::AppendChunk, MemoryPhase::FilterMerged})
  {
    const std::size_t required = go1_mapping::estimated_peak_bytes(phase, state);
    EXPECT_NO_THROW(go1_mapping::enforce_memory_budget(phase, state, required));
    EXPECT_THROW(
      go1_mapping::enforce_memory_budget(phase, state, required - 1U),
      std::length_error);
  }
}

TEST(PcdProjection, RollsBackEveryPublicationTransition)
{
  namespace fs = std::filesystem;
  const std::vector<go1_mapping::PublicationArtifact> artifacts{
    {fs::path{"first.tmp"}, fs::path{"first.pcd"}},
    {fs::path{"second.tmp"}, fs::path{"second.pgm"}},
    {fs::path{"third.tmp"}, fs::path{"third.yaml"}}};

  for (std::size_t fail_at = 0; fail_at < artifacts.size() * 3U; ++fail_at) {
    std::set<fs::path> temporaries{
      artifacts[0].temporary, artifacts[1].temporary, artifacts[2].temporary};
    std::set<fs::path> finals;
    std::size_t transition = 0;
    go1_mapping::PublicationOperations operations;
    operations.link_no_replace = [&](const fs::path &, const fs::path & final_path) {
        if (transition++ == fail_at) {
          throw std::runtime_error("injected link failure");
        }
        finals.insert(final_path);
      };
    operations.unlink_path = [&](const fs::path & path) {
        if (transition++ == fail_at) {
          throw std::runtime_error("injected unlink failure");
        }
        temporaries.erase(path);
        finals.erase(path);
      };
    operations.sync_directory = [&](const fs::path &) {
        if (transition++ == fail_at) {
          throw std::runtime_error("injected sync failure");
        }
      };

    EXPECT_THROW(
      go1_mapping::publish_artifacts(artifacts, operations),
      std::runtime_error) << "transition " << fail_at;
    EXPECT_TRUE(finals.empty()) << "transition " << fail_at;
  }
}

TEST(PcdProjection, PublishesAllArtifactsWithoutAllocatingRollbackState)
{
  namespace fs = std::filesystem;
  const std::vector<go1_mapping::PublicationArtifact> artifacts{
    {fs::path{"first.tmp"}, fs::path{"first.pcd"}},
    {fs::path{"second.tmp"}, fs::path{"second.pgm"}}};
  std::set<fs::path> temporaries{artifacts[0].temporary, artifacts[1].temporary};
  std::set<fs::path> finals;
  go1_mapping::PublicationOperations operations;
  operations.link_no_replace = [&](const fs::path &, const fs::path & final_path) {
      finals.insert(final_path);
    };
  operations.unlink_path = [&](const fs::path & path) {
      temporaries.erase(path);
      finals.erase(path);
    };
  operations.sync_directory = [](const fs::path &) {};

  go1_mapping::publish_artifacts(artifacts, operations);

  EXPECT_TRUE(temporaries.empty());
  EXPECT_EQ(finals.size(), artifacts.size());
}

TEST(PcdProjection, SafelyQuotesRelativeYamlImageFilename)
{
  const std::filesystem::path image_path{"maps/floor 9 # map's.pgm"};

  EXPECT_EQ(
    go1_mapping::yaml_quote(image_path.filename().string()),
    "'floor 9 # map''s.pgm'");
  EXPECT_THROW(go1_mapping::yaml_quote("bad\nname.pgm"), std::invalid_argument);
}

}  // namespace
