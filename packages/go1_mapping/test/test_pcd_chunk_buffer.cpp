#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <stdexcept>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "go1_mapping/pcd_chunk_buffer.hpp"

namespace go1_mapping
{
namespace
{

pcl::PointCloud<pcl::PointXYZI> cloud_with_points(const std::size_t point_count)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.resize(point_count);
  return cloud;
}

TEST(ChunkBuffer, FlushesBeforeFrameLimitIsExceeded)
{
  ChunkBuffer buffer(2, 1024 * 1024);
  const auto cloud = cloud_with_points(1);

  buffer.append(cloud);
  buffer.append(cloud);

  EXPECT_TRUE(buffer.should_flush_before(cloud.size()));
}

TEST(ChunkBuffer, FlushesBeforeByteLimitIsExceeded)
{
  ChunkBuffer buffer(100, sizeof(pcl::PointXYZI) * 2);
  const auto cloud = cloud_with_points(2);

  buffer.append(cloud);

  EXPECT_TRUE(buffer.should_flush_before(1));
}

TEST(ChunkBuffer, DoesNotFlushAnEmptyBuffer)
{
  ChunkBuffer buffer(300, sizeof(pcl::PointXYZI));

  EXPECT_FALSE(buffer.should_flush_before(2));
}

TEST(ChunkBuffer, AllowsAnAppendExactlyAtTheLimits)
{
  ChunkBuffer buffer(1, sizeof(pcl::PointXYZI) * 2);
  const auto cloud = cloud_with_points(2);

  EXPECT_NO_THROW(buffer.append(cloud));
  EXPECT_EQ(buffer.frame_count(), 1U);
  EXPECT_EQ(buffer.byte_count(), sizeof(pcl::PointXYZI) * 2);
}

TEST(ChunkBuffer, RejectsASingleFrameLargerThanTheByteLimit)
{
  ChunkBuffer buffer(300, sizeof(pcl::PointXYZI));
  const auto cloud = cloud_with_points(2);

  EXPECT_THROW(buffer.append(cloud), std::length_error);
  EXPECT_EQ(buffer.frame_count(), 0U);
  EXPECT_EQ(buffer.byte_count(), 0U);
}

TEST(ChunkBuffer, AppendCannotExceedFrameLimit)
{
  ChunkBuffer buffer(1, 1024 * 1024);
  const auto cloud = cloud_with_points(1);
  buffer.append(cloud);

  EXPECT_THROW(buffer.append(cloud), std::length_error);
  EXPECT_EQ(buffer.frame_count(), 1U);
  EXPECT_EQ(buffer.byte_count(), sizeof(pcl::PointXYZI));
}

TEST(ChunkBuffer, AppendCannotExceedByteLimit)
{
  ChunkBuffer buffer(300, sizeof(pcl::PointXYZI) * 2);
  const auto cloud = cloud_with_points(1);
  buffer.append(cloud);

  EXPECT_THROW(buffer.append(cloud_with_points(2)), std::length_error);
  EXPECT_EQ(buffer.frame_count(), 1U);
  EXPECT_EQ(buffer.byte_count(), sizeof(pcl::PointXYZI));
}

TEST(ChunkBuffer, DetectsIncomingPointCountWhoseByteSizeOverflows)
{
  ChunkBuffer buffer(300, std::numeric_limits<std::size_t>::max());
  buffer.append(cloud_with_points(1));

  const auto overflowing_point_count =
    (std::numeric_limits<std::size_t>::max() / sizeof(pcl::PointXYZI)) + 1;
  EXPECT_TRUE(buffer.should_flush_before(overflowing_point_count));
}

TEST(ChunkBuffer, TakeClearsAccounting)
{
  ChunkBuffer buffer(300, 268435456);
  const auto cloud = cloud_with_points(5);

  buffer.append(cloud);
  const auto taken = buffer.take();

  ASSERT_NE(taken, nullptr);
  EXPECT_EQ(taken->size(), 5U);
  EXPECT_EQ(buffer.frame_count(), 0U);
  EXPECT_EQ(buffer.byte_count(), 0U);
}

TEST(ChunkBuffer, EmptyTakeReturnsAnEmptyCloud)
{
  ChunkBuffer buffer(300, 268435456);

  const auto taken = buffer.take();

  ASSERT_NE(taken, nullptr);
  EXPECT_TRUE(taken->empty());
  EXPECT_EQ(buffer.frame_count(), 0U);
  EXPECT_EQ(buffer.byte_count(), 0U);
}

TEST(ChunkBuffer, PeekReturnsAConstViewWithoutClearingAccounting)
{
  ChunkBuffer buffer(300, 268435456);
  buffer.append(cloud_with_points(3));

  const ChunkBuffer & const_buffer = buffer;
  const auto & viewed = const_buffer.peek();

  EXPECT_EQ(viewed.size(), 3U);
  EXPECT_EQ(buffer.frame_count(), 1U);
  EXPECT_EQ(buffer.byte_count(), sizeof(pcl::PointXYZI) * 3);
}

TEST(ChunkBuffer, ClearResetsCloudAndAccounting)
{
  ChunkBuffer buffer(300, 268435456);
  buffer.append(cloud_with_points(3));

  buffer.clear();

  EXPECT_TRUE(buffer.peek().empty());
  EXPECT_EQ(buffer.frame_count(), 0U);
  EXPECT_EQ(buffer.byte_count(), 0U);
}
}  // namespace
}  // namespace go1_mapping
