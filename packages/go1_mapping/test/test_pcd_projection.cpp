#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "go1_mapping/pcd_projection.hpp"

namespace
{

using go1_mapping::Grid;
using go1_mapping::ProjectionBounds;

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

  const auto filtered = go1_mapping::voxel_filter_finite(cloud, 0.10);

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
