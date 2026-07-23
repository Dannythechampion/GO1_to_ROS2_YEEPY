#include <gtest/gtest.h>

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <unistd.h>

#include "go1_mapping/pcd_chunk_buffer.hpp"
#include "go1_mapping/pcd_chunk_storage.hpp"

namespace go1_mapping
{
namespace
{

namespace fs = std::filesystem;

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    static std::atomic<std::size_t> sequence{0};
    path_ = fs::temp_directory_path() /
      ("go1_mapping_storage_test_" + std::to_string(getpid()) + "_" +
      std::to_string(sequence.fetch_add(1)));
    fs::create_directory(path_);
  }

  ~TemporaryDirectory()
  {
    std::error_code error;
    fs::remove_all(path_, error);
  }

  const fs::path & path() const noexcept
  {
    return path_;
  }

private:
  fs::path path_;
};

sensor_msgs::msg::PointCloud2 valid_message()
{
  sensor_msgs::msg::PointCloud2 message;
  message.width = 1;
  message.height = 1;
  message.point_step = 16;
  message.row_step = 16;
  message.data.resize(16);
  message.fields = {
    sensor_msgs::msg::PointField().set__name("x").set__offset(0).set__datatype(
      sensor_msgs::msg::PointField::FLOAT32).set__count(1),
    sensor_msgs::msg::PointField().set__name("y").set__offset(4).set__datatype(
      sensor_msgs::msg::PointField::FLOAT32).set__count(1),
    sensor_msgs::msg::PointField().set__name("z").set__offset(8).set__datatype(
      sensor_msgs::msg::PointField::FLOAT32).set__count(1),
    sensor_msgs::msg::PointField().set__name("intensity").set__offset(12).set__datatype(
      sensor_msgs::msg::PointField::FLOAT32).set__count(1)};
  return message;
}

pcl::PointCloud<pcl::PointXYZI> one_point_cloud()
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI point;
  point.x = 1.0F;
  point.y = 2.0F;
  point.z = 3.0F;
  point.intensity = 4.0F;
  cloud.push_back(point);
  return cloud;
}

void write_text(const fs::path & path, const std::string & text)
{
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(stream.is_open());
  stream << text;
  ASSERT_TRUE(stream.good());
}

std::string read_text(const fs::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}

std::size_t directory_entry_count(const fs::path & directory)
{
  std::size_t count = 0;
  for (const auto & ignored : fs::directory_iterator(directory)) {
    (void)ignored;
    ++count;
  }
  return count;
}

void expect_buffer_preserved(const ChunkBuffer & buffer)
{
  EXPECT_EQ(buffer.frame_count(), 1U);
  EXPECT_EQ(buffer.byte_count(), sizeof(pcl::PointXYZI));
  EXPECT_EQ(buffer.peek().size(), 1U);
}

TEST(CloudMessageValidation, AcceptsAConsistentXYZIFloat32Layout)
{
  const auto result = validate_cloud_message(valid_message(), sizeof(pcl::PointXYZI));

  EXPECT_EQ(result.point_count, 1U);
  EXPECT_EQ(result.payload_bytes, sizeof(pcl::PointXYZI));
}

TEST(CloudMessageValidation, RejectsAMissingRequiredField)
{
  auto message = valid_message();
  message.fields.pop_back();

  EXPECT_THROW(
    validate_cloud_message(message, sizeof(pcl::PointXYZI)), std::invalid_argument);
}

TEST(CloudMessageValidation, RejectsWrongFieldDatatypeOrCount)
{
  auto wrong_datatype = valid_message();
  wrong_datatype.fields.front().datatype = sensor_msgs::msg::PointField::FLOAT64;
  EXPECT_THROW(
    validate_cloud_message(wrong_datatype, sizeof(pcl::PointXYZI)), std::invalid_argument);

  auto wrong_count = valid_message();
  wrong_count.fields.front().count = 2;
  EXPECT_THROW(
    validate_cloud_message(wrong_count, sizeof(pcl::PointXYZI)), std::invalid_argument);
}

TEST(CloudMessageValidation, RejectsFieldOutsidePointStep)
{
  auto message = valid_message();
  message.fields.back().offset = message.point_step;

  EXPECT_THROW(
    validate_cloud_message(message, sizeof(pcl::PointXYZI)), std::invalid_argument);
}

TEST(CloudMessageValidation, RejectsInconsistentRowStep)
{
  auto message = valid_message();
  message.width = 2;

  EXPECT_THROW(
    validate_cloud_message(message, sizeof(pcl::PointXYZI) * 2), std::invalid_argument);
}

TEST(CloudMessageValidation, RejectsDataSizeThatDoesNotMatchRows)
{
  auto message = valid_message();
  message.data.pop_back();

  EXPECT_THROW(
    validate_cloud_message(message, sizeof(pcl::PointXYZI)), std::invalid_argument);
}

TEST(CloudMessageValidation, RejectsEstimatedXYZIPayloadOverTheConfiguredBound)
{
  EXPECT_THROW(
    validate_cloud_message(valid_message(), sizeof(pcl::PointXYZI) - 1), std::length_error);
}

TEST(CloudMessageValidation, RejectsIncomingDataOverTheConfiguredBound)
{
  auto message = valid_message();
  message.point_step = 64;
  message.row_step = 64;
  message.data.resize(64);

  EXPECT_THROW(
    validate_cloud_message(message, sizeof(pcl::PointXYZI)), std::length_error);
}

TEST(CloudMessageValidation, RejectsPointPayloadSizeOverflow)
{
  auto message = valid_message();
  message.width = std::numeric_limits<std::uint32_t>::max();
  message.height = std::numeric_limits<std::uint32_t>::max();

  EXPECT_THROW(
    validate_cloud_message(message, std::numeric_limits<std::size_t>::max()),
    std::length_error);
}

TEST(PcdChunkStorage, EmptyFlushCreatesNoFile)
{
  TemporaryDirectory directory;
  ChunkBuffer buffer(300, 268435456);
  PcdChunkStorage storage(directory.path());

  EXPECT_FALSE(storage.flush(buffer));
  EXPECT_TRUE(fs::is_empty(directory.path()));
}

TEST(PcdChunkStorage, AnonymousPartialHasNoReplaceableDirectoryEntry)
{
  TemporaryDirectory directory;
  ChunkBuffer buffer(300, 268435456);
  buffer.append(one_point_cloud());
  bool saw_anonymous_file = false;
  PcdChunkStorage storage(
    directory.path(),
    [&directory, &saw_anonymous_file](
      const fs::path & descriptor_path, const PcdChunkStorage::Cloud & cloud)
    {
      saw_anonymous_file = fs::is_regular_file(descriptor_path) &&
        fs::file_size(descriptor_path) == 0U && directory_entry_count(directory.path()) == 0U;
      return pcl::io::savePCDFileBinaryCompressed(descriptor_path.string(), cloud);
    });

  EXPECT_TRUE(storage.flush(buffer));

  const auto final_path = directory.path() / "chunk_000000.pcd";
  EXPECT_TRUE(saw_anonymous_file);
  EXPECT_TRUE(fs::is_regular_file(final_path));
  EXPECT_EQ(directory_entry_count(directory.path()), 1U);
  EXPECT_EQ(buffer.frame_count(), 0U);
  EXPECT_EQ(buffer.byte_count(), 0U);
  EXPECT_TRUE(buffer.peek().empty());

  pcl::PointCloud<pcl::PointXYZI> loaded;
  ASSERT_EQ(pcl::io::loadPCDFile(final_path.string(), loaded), 0);
  ASSERT_EQ(loaded.size(), 1U);
  EXPECT_FLOAT_EQ(loaded.front().intensity, 4.0F);
}

TEST(PcdChunkStorage, SaveFailureLeavesNoEntryAndPreservesBuffer)
{
  TemporaryDirectory directory;
  ChunkBuffer buffer(300, 268435456);
  buffer.append(one_point_cloud());
  PcdChunkStorage storage(
    directory.path(),
    [&directory](const fs::path & descriptor_path, const PcdChunkStorage::Cloud &)
    {
      EXPECT_EQ(directory_entry_count(directory.path()), 0U);
      write_text(descriptor_path, "incomplete");
      return -1;
    });

  EXPECT_THROW(storage.flush(buffer), std::runtime_error);

  EXPECT_EQ(directory_entry_count(directory.path()), 0U);
  expect_buffer_preserved(buffer);
}

TEST(PcdChunkStorage, ExistingFinalIsNeverClobberedAndNextIndexIsUsed)
{
  TemporaryDirectory directory;
  const auto existing_path = directory.path() / "chunk_000000.pcd";
  write_text(existing_path, "original bytes");
  ChunkBuffer buffer(300, 268435456);
  buffer.append(one_point_cloud());
  PcdChunkStorage storage(directory.path());

  EXPECT_TRUE(storage.flush(buffer));

  EXPECT_EQ(read_text(existing_path), "original bytes");
  EXPECT_TRUE(fs::is_regular_file(directory.path() / "chunk_000001.pcd"));
  EXPECT_EQ(directory_entry_count(directory.path()), 2U);
}

TEST(PcdChunkStorage, ConcurrentFinalCollisionPreservesBytesAndBuffer)
{
  TemporaryDirectory directory;
  ChunkBuffer buffer(300, 268435456);
  buffer.append(one_point_cloud());
  const auto colliding_final = directory.path() / "chunk_000000.pcd";
  PcdChunkStorage storage(
    directory.path(),
    [&colliding_final](const fs::path & descriptor_path, const PcdChunkStorage::Cloud &)
    {
      write_text(descriptor_path, "new bytes");
      write_text(colliding_final, "concurrent bytes");
      return 0;
    });

  EXPECT_THROW(storage.flush(buffer), std::runtime_error);

  EXPECT_EQ(read_text(colliding_final), "concurrent bytes");
  EXPECT_EQ(directory_entry_count(directory.path()), 1U);
  expect_buffer_preserved(buffer);
}

TEST(PcdChunkStorage, DirectorySyncHappensBeforeBufferClearAndSuccess)
{
  TemporaryDirectory directory;
  ChunkBuffer buffer(300, 268435456);
  buffer.append(one_point_cloud());
  std::size_t sync_calls = 0;
  PcdChunkStorageOperations operations;
  operations.sync_directory = [&directory, &buffer, &sync_calls](int)
    {
      ++sync_calls;
      EXPECT_TRUE(fs::exists(directory.path() / "chunk_000000.pcd"));
      EXPECT_EQ(directory_entry_count(directory.path()), 1U);
      expect_buffer_preserved(buffer);
    };
  PcdChunkStorage storage(
    directory.path(),
    [](const fs::path & descriptor_path, const PcdChunkStorage::Cloud &)
    {
      write_text(descriptor_path, "complete bytes");
      return 0;
    },
    operations);

  EXPECT_TRUE(storage.flush(buffer));
  EXPECT_EQ(sync_calls, 1U);
  EXPECT_TRUE(buffer.peek().empty());
}

TEST(PcdChunkStorage, UnsupportedAnonymousTemporaryFileFailsExplicitly)
{
  TemporaryDirectory directory;
  ChunkBuffer buffer(300, 268435456);
  buffer.append(one_point_cloud());
  PcdChunkStorageOperations operations;
  operations.open_anonymous = [](int) -> int
    {
      throw std::system_error(
              EOPNOTSUPP, std::generic_category(), "O_TMPFILE is unsupported");
    };
  PcdChunkStorage storage(directory.path(), PcdChunkStorage::SaveFunction{}, operations);

  EXPECT_THROW(storage.flush(buffer), std::system_error);
  EXPECT_EQ(directory_entry_count(directory.path()), 0U);
  expect_buffer_preserved(buffer);
}

TEST(PcdChunkStorage, AnonymousFileStatusFailureLeavesNoEntryAndPreservesBuffer)
{
  TemporaryDirectory directory;
  ChunkBuffer buffer(300, 268435456);
  buffer.append(one_point_cloud());
  PcdChunkStorageOperations operations;
  operations.validate_anonymous = [](int)
    {
      throw std::system_error(EIO, std::generic_category(), "injected fstat failure");
    };
  PcdChunkStorage storage(
    directory.path(),
    [](const fs::path & descriptor_path, const PcdChunkStorage::Cloud &)
    {
      write_text(descriptor_path, "complete bytes");
      return 0;
    },
    operations);

  EXPECT_THROW(storage.flush(buffer), std::system_error);
  EXPECT_EQ(directory_entry_count(directory.path()), 0U);
  expect_buffer_preserved(buffer);
}

TEST(PcdChunkStorage, AnonymousFileSyncFailureLeavesNoEntryAndPreservesBuffer)
{
  TemporaryDirectory directory;
  ChunkBuffer buffer(300, 268435456);
  buffer.append(one_point_cloud());
  PcdChunkStorageOperations operations;
  operations.sync_file = [](int)
    {
      throw std::system_error(EIO, std::generic_category(), "injected file fsync failure");
    };
  PcdChunkStorage storage(
    directory.path(),
    [](const fs::path & descriptor_path, const PcdChunkStorage::Cloud &)
    {
      write_text(descriptor_path, "complete bytes");
      return 0;
    },
    operations);

  EXPECT_THROW(storage.flush(buffer), std::system_error);
  EXPECT_EQ(directory_entry_count(directory.path()), 0U);
  expect_buffer_preserved(buffer);
}

TEST(PcdChunkStorage, PublishFailureLeavesNoEntryAndPreservesBuffer)
{
  TemporaryDirectory directory;
  ChunkBuffer buffer(300, 268435456);
  buffer.append(one_point_cloud());
  PcdChunkStorageOperations operations;
  operations.publish = [](int, int, const std::string &)
    {
      throw std::system_error(EOPNOTSUPP, std::generic_category(), "AT_EMPTY_PATH unsupported");
    };
  PcdChunkStorage storage(
    directory.path(),
    [](const fs::path & descriptor_path, const PcdChunkStorage::Cloud &)
    {
      write_text(descriptor_path, "complete bytes");
      return 0;
    },
    operations);

  EXPECT_THROW(storage.flush(buffer), std::system_error);
  EXPECT_EQ(directory_entry_count(directory.path()), 0U);
  expect_buffer_preserved(buffer);
}

TEST(PcdChunkStorage, DirectorySyncFailureKeepsCompleteFinalAndRetryUsesNextIndex)
{
  TemporaryDirectory directory;
  ChunkBuffer buffer(300, 268435456);
  buffer.append(one_point_cloud());
  PcdChunkStorageOperations operations;
  operations.sync_directory = [](int)
    {
      throw std::system_error(EIO, std::generic_category(), "injected directory fsync failure");
    };
  PcdChunkStorage failing_storage(
    directory.path(),
    [](const fs::path & descriptor_path, const PcdChunkStorage::Cloud &)
    {
      write_text(descriptor_path, "complete first bytes");
      return 0;
    },
    operations);

  EXPECT_THROW(failing_storage.flush(buffer), std::system_error);

  const auto first_final = directory.path() / "chunk_000000.pcd";
  EXPECT_EQ(read_text(first_final), "complete first bytes");
  EXPECT_EQ(directory_entry_count(directory.path()), 1U);
  expect_buffer_preserved(buffer);

  PcdChunkStorage retry_storage(
    directory.path(),
    [](const fs::path & descriptor_path, const PcdChunkStorage::Cloud &)
    {
      write_text(descriptor_path, "retry bytes");
      return 0;
    });
  EXPECT_TRUE(retry_storage.flush(buffer));

  EXPECT_EQ(read_text(first_final), "complete first bytes");
  EXPECT_EQ(read_text(directory.path() / "chunk_000001.pcd"), "retry bytes");
  EXPECT_EQ(directory_entry_count(directory.path()), 2U);
  EXPECT_TRUE(buffer.peek().empty());
}
}  // namespace
}  // namespace go1_mapping
