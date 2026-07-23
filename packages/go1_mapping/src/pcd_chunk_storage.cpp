#include "go1_mapping/pcd_chunk_storage.hpp"

#include <array>
#include <atomic>
#include <cerrno>
#include <cstdint>
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

void sync_file(const fs::path & path)
{
  const int descriptor = open(path.c_str(), O_RDONLY | O_CLOEXEC);
  if (descriptor < 0) {
    throw std::system_error(errno, std::generic_category(), "open partial PCD for fsync");
  }
  const int sync_status = fsync(descriptor);
  const int sync_error = errno;
  const int close_status = close(descriptor);
  const int close_error = errno;
  if (sync_status != 0) {
    throw std::system_error(sync_error, std::generic_category(), "fsync partial PCD");
  }
  if (close_status != 0) {
    throw std::system_error(close_error, std::generic_category(), "close synced partial PCD");
  }
}

class PartialFileGuard
{
public:
  explicit PartialFileGuard(fs::path path)
  : path_(std::move(path)) {}

  ~PartialFileGuard()
  {
    if (active_) {
      std::error_code ignored;
      fs::remove(path_, ignored);
    }
  }

  void release() noexcept
  {
    active_ = false;
  }

private:
  fs::path path_;
  bool active_{true};
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

PcdChunkStorage::PcdChunkStorage(fs::path output_dir, SaveFunction save_function)
: output_dir_(std::move(output_dir)),
  save_function_(std::move(save_function))
{
  if (!fs::is_directory(output_dir_)) {
    throw std::invalid_argument("PCD output directory does not exist");
  }
  if (!save_function_) {
    save_function_ = [](const fs::path & path, const Cloud & cloud) {
        return pcl::io::savePCDFileBinaryCompressed(path.string(), cloud);
      };
  }
}

std::size_t PcdChunkStorage::next_free_index() const
{
  std::size_t index = 0;
  while (fs::exists(output_dir_ / (chunk_stem(index) + ".pcd"))) {
    if (index == std::numeric_limits<std::size_t>::max()) {
      throw std::overflow_error("no PCD chunk index is available");
    }
    ++index;
  }
  return index;
}

fs::path PcdChunkStorage::reserve_unique_partial(const std::size_t index) const
{
  static std::atomic<std::uint64_t> sequence{0};
  for (std::size_t attempt = 0; attempt < 128; ++attempt) {
    const auto ticket = sequence.fetch_add(1, std::memory_order_relaxed);
    const fs::path partial_path = output_dir_ /
      ("." + chunk_stem(index) + "." + std::to_string(getpid()) + "." +
      std::to_string(ticket) + ".partial");
    const int descriptor = open(
      partial_path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if (descriptor >= 0) {
      if (close(descriptor) != 0) {
        const int close_error = errno;
        std::error_code ignored;
        fs::remove(partial_path, ignored);
        throw std::system_error(close_error, std::generic_category(), "close partial PCD");
      }
      return partial_path;
    }
    if (errno != EEXIST) {
      throw std::system_error(errno, std::generic_category(), "reserve partial PCD");
    }
  }
  throw std::runtime_error("could not reserve a unique partial PCD path");
}

bool PcdChunkStorage::flush(ChunkBuffer & buffer)
{
  if (buffer.peek().empty()) {
    buffer.clear();
    return false;
  }

  const auto index = next_free_index();
  const fs::path final_path = output_dir_ / (chunk_stem(index) + ".pcd");
  const fs::path partial_path = reserve_unique_partial(index);
  PartialFileGuard partial_guard(partial_path);

  if (save_function_(partial_path, buffer.peek()) < 0) {
    throw std::runtime_error("failed to save binary-compressed partial PCD");
  }

  sync_file(partial_path);

  std::error_code publish_error;
  fs::create_hard_link(partial_path, final_path, publish_error);
  if (publish_error) {
    throw fs::filesystem_error(
            "atomic no-clobber PCD publish failed", partial_path, final_path, publish_error);
  }

  std::error_code cleanup_error;
  fs::remove(partial_path, cleanup_error);
  buffer.clear();
  if (cleanup_error) {
    throw fs::filesystem_error("published PCD but could not remove partial", partial_path,
            cleanup_error);
  }
  partial_guard.release();
  return true;
}

}  // namespace go1_mapping
