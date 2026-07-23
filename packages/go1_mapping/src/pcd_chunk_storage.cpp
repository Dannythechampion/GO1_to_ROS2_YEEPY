#include "go1_mapping/pcd_chunk_storage.hpp"

#include <array>
#include <cerrno>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_field.hpp>
#include <sys/stat.h>
#include <unistd.h>

namespace go1_mapping
{
namespace
{

namespace fs = std::filesystem;

std::size_t checked_multiply(
  const std::size_t left,
  const std::size_t right,
  const char * const description)
{
  if (left != 0 && right > std::numeric_limits<std::size_t>::max() / left) {
    throw std::length_error(std::string(description) + " overflows size_t");
  }
  return left * right;
}

std::string chunk_stem(const std::size_t index)
{
  std::ostringstream name;
  name << "chunk_" << std::setfill('0') << std::setw(6) << index;
  return name.str();
}

void sync_descriptor(const int descriptor, const char * const description)
{
  if (fsync(descriptor) != 0) {
    throw std::system_error(errno, std::generic_category(), description);
  }
}

class OwnedDescriptor
{
public:
  explicit OwnedDescriptor(const int descriptor)
  : descriptor_(descriptor) {}

  ~OwnedDescriptor()
  {
    if (descriptor_ >= 0) {
      close(descriptor_);
    }
  }

  OwnedDescriptor(const OwnedDescriptor &) = delete;
  OwnedDescriptor & operator=(const OwnedDescriptor &) = delete;

  int get() const noexcept
  {
    return descriptor_;
  }

private:
  int descriptor_;
};

}  // namespace

ValidatedCloudMessage validate_cloud_message(
  const sensor_msgs::msg::PointCloud2 & message,
  const std::size_t max_payload_bytes)
{
  if (max_payload_bytes == 0) {
    throw std::invalid_argument("max_payload_bytes must be greater than zero");
  }

  const auto point_count = checked_multiply(
    static_cast<std::size_t>(message.width),
    static_cast<std::size_t>(message.height),
    "width * height");
  const auto payload_bytes = checked_multiply(
    point_count, sizeof(pcl::PointXYZI), "PointXYZI payload");
  if (payload_bytes > max_payload_bytes) {
    throw std::length_error("estimated PointXYZI payload exceeds max_payload_bytes");
  }
  if (message.data.size() > max_payload_bytes) {
    throw std::length_error("serialized PointCloud2 data exceeds max_payload_bytes");
  }

  if (point_count != 0 && message.point_step == 0) {
    throw std::invalid_argument("point_step must be greater than zero for a non-empty cloud");
  }
  const auto minimum_row_step = checked_multiply(
    static_cast<std::size_t>(message.point_step),
    static_cast<std::size_t>(message.width),
    "point_step * width");
  if (static_cast<std::size_t>(message.row_step) < minimum_row_step) {
    throw std::invalid_argument("row_step is smaller than point_step * width");
  }
  const auto expected_data_size = checked_multiply(
    static_cast<std::size_t>(message.row_step),
    static_cast<std::size_t>(message.height),
    "row_step * height");
  if (message.data.size() != expected_data_size) {
    throw std::invalid_argument("data size does not equal row_step * height");
  }

  constexpr std::array<std::string_view, 4> required_fields = {
    "x", "y", "z", "intensity"};
  for (const auto required_name : required_fields) {
    const sensor_msgs::msg::PointField * matched_field = nullptr;
    for (const auto & field : message.fields) {
      if (field.name == required_name) {
        if (matched_field != nullptr) {
          throw std::invalid_argument("duplicate required PointCloud2 field: " + field.name);
        }
        matched_field = &field;
      }
    }
    if (matched_field == nullptr) {
      throw std::invalid_argument(
              "missing required PointCloud2 field: " + std::string(required_name));
    }
    if (matched_field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      matched_field->count != 1)
    {
      throw std::invalid_argument(
              "required PointCloud2 fields must be FLOAT32 with count 1");
    }
    const auto field_end = checked_multiply(
      static_cast<std::size_t>(matched_field->count), sizeof(float), "field byte size");
    if (static_cast<std::size_t>(matched_field->offset) >
      std::numeric_limits<std::size_t>::max() - field_end)
    {
      throw std::length_error("PointCloud2 field offset overflows size_t");
    }
    if (static_cast<std::size_t>(matched_field->offset) + field_end > message.point_step) {
      throw std::invalid_argument("required PointCloud2 field exceeds point_step");
    }
  }

  return ValidatedCloudMessage{point_count, payload_bytes};
}

PcdChunkStorage::PcdChunkStorage(
  fs::path output_dir,
  SaveFunction save_function,
  PcdChunkStorageOperations operations)
: output_dir_(std::move(output_dir)),
  save_function_(std::move(save_function)),
  operations_(std::move(operations))
{
  if (!save_function_) {
    save_function_ = [](const fs::path & descriptor_path, const Cloud & cloud) {
        return pcl::io::savePCDFileBinaryCompressed(descriptor_path.string(), cloud);
      };
  }
  if (!operations_.open_anonymous) {
    operations_.open_anonymous = [](const int directory_fd) {
        const int descriptor = openat(
          directory_fd, ".", O_TMPFILE | O_RDWR | O_CLOEXEC, 0644);
        if (descriptor < 0) {
          throw std::system_error(
                  errno, std::generic_category(),
                  "openat O_TMPFILE PCD (Linux filesystem support required)");
        }
        return descriptor;
      };
  }
  if (!operations_.validate_anonymous) {
    operations_.validate_anonymous = [](const int descriptor) {
        struct stat status {};
        if (fstat(descriptor, &status) != 0) {
          throw std::system_error(errno, std::generic_category(), "fstat anonymous PCD");
        }
        if (!S_ISREG(status.st_mode) || status.st_nlink != 0) {
          throw std::runtime_error("O_TMPFILE did not produce an unlinked regular PCD inode");
        }
      };
  }
  if (!operations_.sync_file) {
    operations_.sync_file = [](const int descriptor) {
        sync_descriptor(descriptor, "fsync anonymous PCD");
      };
  }
  if (!operations_.publish) {
    operations_.publish = [](
      const int descriptor, const int directory_fd, const std::string & final_name)
      {
        if (linkat(descriptor, "", directory_fd, final_name.c_str(), AT_EMPTY_PATH) != 0) {
          throw std::system_error(
                  errno, std::generic_category(),
                  "linkat AT_EMPTY_PATH no-clobber PCD publish");
        }
      };
  }
  if (!operations_.sync_directory) {
    operations_.sync_directory = [](const int descriptor) {
        sync_descriptor(
          descriptor,
          "fsync PCD output directory; complete final may exist but durability is unconfirmed"
        );
      };
  }

  output_dir_fd_ = open(
    output_dir_.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
  if (output_dir_fd_ < 0) {
    throw std::system_error(errno, std::generic_category(), "open PCD output directory");
  }
}

PcdChunkStorage::~PcdChunkStorage()
{
  if (output_dir_fd_ >= 0) {
    close(output_dir_fd_);
  }
}

std::size_t PcdChunkStorage::next_free_index() const
{
  std::size_t index = 0;
  while (true) {
    struct stat status {};
    const auto final_name = chunk_stem(index) + ".pcd";
    if (fstatat(output_dir_fd_, final_name.c_str(), &status, AT_SYMLINK_NOFOLLOW) != 0) {
      if (errno == ENOENT) {
        return index;
      }
      throw std::system_error(errno, std::generic_category(), "fstatat final PCD");
    }
    if (index == std::numeric_limits<std::size_t>::max()) {
      throw std::overflow_error("no PCD chunk index is available");
    }
    ++index;
  }
}

bool PcdChunkStorage::flush(ChunkBuffer & buffer)
{
  if (buffer.peek().empty()) {
    buffer.clear();
    return false;
  }

  const auto index = next_free_index();
  const std::string final_name = chunk_stem(index) + ".pcd";
  const int anonymous_descriptor = operations_.open_anonymous(output_dir_fd_);
  if (anonymous_descriptor < 0) {
    throw std::runtime_error("anonymous PCD opener returned an invalid descriptor");
  }
  OwnedDescriptor anonymous_file(anonymous_descriptor);
  const fs::path descriptor_path =
    fs::path("/proc/self/fd") / std::to_string(anonymous_file.get());

  if (save_function_(descriptor_path, buffer.peek()) < 0) {
    throw std::runtime_error("failed to save binary-compressed anonymous PCD");
  }
  operations_.validate_anonymous(anonymous_file.get());
  operations_.sync_file(anonymous_file.get());
  operations_.publish(anonymous_file.get(), output_dir_fd_, final_name);
  operations_.sync_directory(output_dir_fd_);
  buffer.clear();
  return true;
}

}  // namespace go1_mapping
