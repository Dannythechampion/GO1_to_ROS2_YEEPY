#include "go1_mapping/pcd_projection.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include <pcl/filters/voxel_grid.h>

namespace go1_mapping
{
namespace
{

constexpr double kPaddingMeters = 1.0;
constexpr std::uint8_t kUnknown = 205;
constexpr std::uint8_t kOccupied = 0;
constexpr std::size_t kMaximumGridCells = 100000000;

bool finite_point(const pcl::PointXYZI & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

std::size_t checked_dimension(const double extent, const double resolution)
{
  const double cells = std::ceil(extent / resolution);
  if (!std::isfinite(cells) || cells < 1.0 ||
    cells > static_cast<double>(std::numeric_limits<std::size_t>::max()))
  {
    throw std::length_error("projected grid dimension is invalid or too large");
  }
  return static_cast<std::size_t>(cells);
}

std::size_t checked_add_bytes(const std::size_t left, const std::size_t right)
{
  if (right > std::numeric_limits<std::size_t>::max() - left) {
    throw std::length_error("aggregate memory estimate overflows size_t");
  }
  return left + right;
}

std::size_t checked_multiply_bytes(const std::size_t left, const std::size_t right)
{
  if (left != 0U && right > std::numeric_limits<std::size_t>::max() / left) {
    throw std::length_error("aggregate memory estimate overflows size_t");
  }
  return left * right;
}

std::size_t parse_size_token(const std::string & token, const char * const field)
{
  if (token.empty()) {
    throw std::invalid_argument(std::string("PCD header missing value for ") + field);
  }
  std::size_t value = 0;
  const auto result = std::from_chars(token.data(), token.data() + token.size(), value);
  if (result.ec != std::errc{} || result.ptr != token.data() + token.size()) {
    throw std::invalid_argument(std::string("invalid PCD header value for ") + field);
  }
  return value;
}

void remove_nonfinite_xyz(pcl::PointCloud<pcl::PointXYZI> & cloud)
{
  const auto new_end = std::remove_if(
    cloud.points.begin(), cloud.points.end(),
    [](const pcl::PointXYZI & point) {return !finite_point(point);});
  cloud.points.erase(new_end, cloud.points.end());
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1U;
  cloud.is_dense = true;
}

}  // namespace

std::size_t point_payload_bytes(const std::size_t point_count)
{
  return checked_multiply_bytes(point_count, sizeof(pcl::PointXYZI));
}

std::size_t checked_merged_point_count(
  const std::size_t existing_points,
  const std::size_t incoming_points,
  const std::size_t max_bytes)
{
  if (max_bytes < sizeof(pcl::PointXYZI)) {
    throw std::invalid_argument("merged cloud byte ceiling is too small for one point");
  }
  const std::size_t max_points = max_bytes / sizeof(pcl::PointXYZI);
  if (existing_points > max_points || incoming_points > max_points - existing_points) {
    throw std::length_error("merged cloud exceeds the 256 MiB point payload ceiling");
  }
  return existing_points + incoming_points;
}

std::size_t estimated_peak_bytes(
  const MemoryPhase phase,
  const AggregateMemoryState state)
{
  switch (phase) {
    case MemoryPhase::LoadChunk:
      return checked_add_bytes(
        checked_add_bytes(
          checked_add_bytes(state.merged_bytes, state.incoming_bytes),
          state.raw_decoded_bytes),
        state.file_bytes);
    case MemoryPhase::FilterChunk:
      return checked_add_bytes(
        state.merged_bytes,
        checked_multiply_bytes(kVoxelFilterPayloadCopies, state.incoming_bytes));
    case MemoryPhase::AppendChunk:
      return checked_add_bytes(
        checked_add_bytes(state.merged_bytes, state.incoming_bytes),
        state.combined_bytes);
    case MemoryPhase::FilterMerged:
      return checked_multiply_bytes(kVoxelFilterPayloadCopies, state.merged_bytes);
  }
  throw std::invalid_argument("unknown aggregate memory phase");
}

void enforce_memory_budget(
  const MemoryPhase phase,
  const AggregateMemoryState state,
  const std::size_t budget_bytes)
{
  if (estimated_peak_bytes(phase, state) > budget_bytes) {
    throw std::length_error("modeled concurrent PCD memory exceeds the process budget");
  }
}

PcdHeaderMetadata preflight_pcd_header(
  std::istream & input,
  const std::size_t file_bytes,
  const std::size_t retained_merged_bytes,
  const std::size_t budget_bytes)
{
  constexpr std::size_t kMaximumHeaderBytes = 65536U;
  constexpr std::size_t kMaximumLineBytes = 4096U;
  std::vector<std::string> field_names;
  std::vector<std::size_t> field_sizes;
  std::vector<std::string> field_types;
  std::vector<std::size_t> field_counts;
  std::optional<std::size_t> width;
  std::optional<std::size_t> height;
  std::optional<std::size_t> points;
  std::optional<PcdDataEncoding> encoding;
  bool found_data = false;
  std::size_t header_bytes = 0;
  std::string line;
  line.reserve(256U);

  const auto consume_line = [&](std::string value) {
      if (!value.empty() && value.back() == '\r') {
        value.pop_back();
      }
      std::istringstream fields(value);
      std::string key;
      fields >> key;
      if (key.empty() || key.front() == '#') {
        return false;
      }
      std::vector<std::string> tokens;
      std::string token;
      while (fields >> token) {
        tokens.push_back(token);
      }
      const auto require_unique_declaration = [&](const bool already_set) {
          if (already_set) {
            throw std::invalid_argument("duplicate PCD header declaration: " + key);
          }
          if (tokens.empty()) {
            throw std::invalid_argument("empty PCD header declaration: " + key);
          }
        };
      if (key == "FIELDS") {
        require_unique_declaration(!field_names.empty());
        field_names = tokens;
      } else if (key == "SIZE") {
        require_unique_declaration(!field_sizes.empty());
        for (const auto & item : tokens) {
          field_sizes.push_back(parse_size_token(item, "SIZE"));
        }
      } else if (key == "TYPE") {
        require_unique_declaration(!field_types.empty());
        field_types = tokens;
      } else if (key == "COUNT") {
        require_unique_declaration(!field_counts.empty());
        for (const auto & item : tokens) {
          field_counts.push_back(parse_size_token(item, "COUNT"));
        }
      } else if (key == "WIDTH" || key == "HEIGHT" || key == "POINTS") {
        require_unique_declaration(
          key == "WIDTH" ? width.has_value() :
          (key == "HEIGHT" ? height.has_value() : points.has_value()));
        if (tokens.size() != 1U) {
          throw std::invalid_argument("PCD scalar declaration has multiple values: " + key);
        }
        const std::size_t parsed = parse_size_token(tokens.front(), key.c_str());
        if (key == "WIDTH") {
          width = parsed;
        } else if (key == "HEIGHT") {
          height = parsed;
        } else {
          points = parsed;
        }
      } else if (key == "DATA") {
        require_unique_declaration(encoding.has_value());
        if (tokens.size() != 1U) {
          throw std::invalid_argument("PCD DATA must have exactly one encoding");
        }
        if (tokens.front() == "ascii") {
          encoding = PcdDataEncoding::Ascii;
        } else if (tokens.front() == "binary") {
          encoding = PcdDataEncoding::Binary;
        } else if (tokens.front() == "binary_compressed") {
          encoding = PcdDataEncoding::BinaryCompressed;
        } else {
          throw std::invalid_argument("unsupported PCD DATA encoding");
        }
        return true;
      }
      return false;
    };

  char character = '\0';
  while (input.get(character)) {
    if (++header_bytes > kMaximumHeaderBytes) {
      throw std::length_error("PCD header exceeds 64 KiB");
    }
    if (character == '\n') {
      found_data = consume_line(line);
      line.clear();
      if (found_data) {
        break;
      }
    } else {
      if (line.size() >= kMaximumLineBytes) {
        throw std::length_error("PCD header line exceeds 4 KiB");
      }
      line.push_back(character);
    }
  }
  if (!found_data && !line.empty()) {
    found_data = consume_line(line);
  }
  if (!found_data || !encoding) {
    throw std::invalid_argument("PCD header has no supported DATA declaration");
  }
  if (file_bytes < header_bytes) {
    throw std::invalid_argument("PCD file is shorter than its parsed header");
  }

  const std::size_t field_count = field_names.size();
  if (field_count == 0U || field_sizes.size() != field_count ||
    field_types.size() != field_count || field_counts.size() != field_count)
  {
    throw std::invalid_argument("PCD FIELDS/SIZE/TYPE/COUNT lengths must match");
  }
  std::size_t raw_point_step = 0U;
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  bool has_intensity = false;
  for (std::size_t index = 0; index < field_count; ++index) {
    if (field_sizes[index] == 0U || field_counts[index] == 0U) {
      throw std::invalid_argument("PCD SIZE and COUNT values must be positive");
    }
    for (std::size_t previous = 0; previous < index; ++previous) {
      if (field_names[index] == field_names[previous]) {
        throw std::invalid_argument("duplicate PCD field name");
      }
    }
    if (field_types[index].size() != 1U ||
      (field_types[index] != "F" && field_types[index] != "I" &&
      field_types[index] != "U"))
    {
      throw std::invalid_argument("unsupported PCD field TYPE");
    }
    const bool supported_size = field_types[index] == "F" ?
      (field_sizes[index] == 4U || field_sizes[index] == 8U) :
      (field_sizes[index] == 1U || field_sizes[index] == 2U ||
      field_sizes[index] == 4U || field_sizes[index] == 8U);
    if (!supported_size) {
      throw std::invalid_argument("unsupported PCD field SIZE for TYPE");
    }
    const std::size_t field_bytes = checked_multiply_bytes(
      field_sizes[index], field_counts[index]);
    raw_point_step = checked_add_bytes(raw_point_step, field_bytes);
    const bool required_compatible = field_types[index] == "F" &&
      field_sizes[index] == 4U && field_counts[index] == 1U;
    if (field_names[index] == "x") {
      has_x = required_compatible;
    } else if (field_names[index] == "y") {
      has_y = required_compatible;
    } else if (field_names[index] == "z") {
      has_z = required_compatible;
    } else if (field_names[index] == "intensity") {
      has_intensity = required_compatible;
    }
  }
  if (!has_x || !has_y || !has_z || !has_intensity) {
    throw std::invalid_argument("PCD x/y/z/intensity must be FLOAT32 with COUNT 1");
  }

  std::optional<std::size_t> dimensions;
  if (width.has_value() != height.has_value()) {
    throw std::invalid_argument("PCD WIDTH and HEIGHT must appear together");
  }
  if (width && height) {
    dimensions = checked_multiply_bytes(*width, *height);
  }
  if (points && dimensions && *points != *dimensions) {
    throw std::invalid_argument("PCD POINTS does not equal WIDTH * HEIGHT");
  }
  const std::size_t point_count = points ? *points :
    (dimensions ? *dimensions : throw std::invalid_argument("PCD point count is missing"));
  checked_merged_point_count(0U, point_count);
  const std::size_t raw_decoded_bytes = checked_multiply_bytes(raw_point_step, point_count);
  if (raw_decoded_bytes > kMaximumMergedCloudBytes) {
    throw std::length_error("raw decoded PCD layout exceeds 256 MiB");
  }
  const std::size_t decoded_bytes = point_payload_bytes(point_count);
  std::size_t compressed_bytes = 0U;
  if (*encoding == PcdDataEncoding::Ascii) {
    if (point_count > 0U && file_bytes == header_bytes) {
      throw std::invalid_argument("ASCII PCD payload is truncated");
    }
  } else if (*encoding == PcdDataEncoding::Binary) {
    const std::size_t expected_file_bytes = checked_add_bytes(header_bytes, raw_decoded_bytes);
    if (file_bytes != expected_file_bytes) {
      throw std::invalid_argument("binary PCD payload length does not match its layout");
    }
  } else {
    std::array<unsigned char, 8> prefix{};
    input.read(reinterpret_cast<char *>(prefix.data()), prefix.size());
    if (input.gcount() != static_cast<std::streamsize>(prefix.size())) {
      throw std::invalid_argument("binary_compressed PCD size prefix is truncated");
    }
    const auto little_u32 = [&](const std::size_t offset) {
        std::uint32_t value = 0U;
        for (std::size_t index = 0; index < 4U; ++index) {
          value |= static_cast<std::uint32_t>(prefix[offset + index]) << (index * 8U);
        }
        return value;
      };
    compressed_bytes = little_u32(0U);
    const std::size_t uncompressed_bytes = little_u32(4U);
    if (compressed_bytes > kMaximumMergedCloudBytes) {
      throw std::length_error("compressed PCD payload exceeds 256 MiB");
    }
    if (uncompressed_bytes != raw_decoded_bytes) {
      throw std::invalid_argument(
              "binary_compressed uncompressed_size does not match PCD layout");
    }
    const std::size_t expected_file_bytes = checked_add_bytes(
      checked_add_bytes(header_bytes, prefix.size()), compressed_bytes);
    if (file_bytes != expected_file_bytes) {
      throw std::invalid_argument(
              "binary_compressed compressed_size does not match file length");
    }
  }

  enforce_memory_budget(
    MemoryPhase::LoadChunk,
    AggregateMemoryState{
      retained_merged_bytes, decoded_bytes, file_bytes, 0U, raw_decoded_bytes},
    budget_bytes);
  return PcdHeaderMetadata{
    *encoding, point_count, raw_point_step, raw_decoded_bytes, decoded_bytes,
    compressed_bytes, header_bytes, file_bytes};
}

pcl::PointCloud<pcl::PointXYZI> voxel_filter_finite(
  pcl::PointCloud<pcl::PointXYZI> && transferred_cloud,
  const double voxel_size,
  const std::size_t retained_merged_bytes,
  const std::size_t budget_bytes)
{
  const float leaf = static_cast<float>(voxel_size);
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0 ||
    !std::isfinite(leaf) || leaf <= 0.0F)
  {
    throw std::invalid_argument("voxel size is outside the supported positive float range");
  }
  checked_merged_point_count(0U, transferred_cloud.capacity());
  const std::size_t input_bytes = point_payload_bytes(transferred_cloud.capacity());
  enforce_memory_budget(
    MemoryPhase::FilterChunk,
    AggregateMemoryState{retained_merged_bytes, input_bytes, 0U, 0U, 0U},
    budget_bytes);
  pcl::PointCloud<pcl::PointXYZI> cloud = std::move(transferred_cloud);
  remove_nonfinite_xyz(cloud);
  if (cloud.empty()) {
    return cloud;
  }
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr input_pointer(
    &cloud, [](const pcl::PointCloud<pcl::PointXYZI> *) {});
  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setInputCloud(input_pointer);
  filter.setLeafSize(leaf, leaf, leaf);
  pcl::PointCloud<pcl::PointXYZI> output;
  filter.filter(output);
  checked_merged_point_count(0U, output.capacity());
  remove_nonfinite_xyz(output);
  return output;
}

void publish_artifacts(
  const std::vector<PublicationArtifact> & artifacts,
  const PublicationOperations & operations)
{
  if (artifacts.empty()) {
    throw std::invalid_argument("at least one artifact is required for publication");
  }
  if (!operations.link_no_replace || !operations.unlink_path ||
    !operations.sync_directory)
  {
    throw std::invalid_argument("all publication operations are required");
  }
  for (std::size_t left = 0; left < artifacts.size(); ++left) {
    if (artifacts[left].temporary.empty() || artifacts[left].final.empty() ||
      artifacts[left].temporary == artifacts[left].final)
    {
      throw std::invalid_argument("publication paths must be non-empty and distinct");
    }
    for (std::size_t right = left + 1; right < artifacts.size(); ++right) {
      if (artifacts[left].final == artifacts[right].final) {
        throw std::invalid_argument("publication final paths must be unique");
      }
    }
  }

  std::size_t published_count = 0;
  try {
    for (const auto & artifact : artifacts) {
      operations.link_no_replace(artifact.temporary, artifact.final);
      // A built-in counter cannot allocate or throw: the new final is tracked immediately.
      ++published_count;
      operations.unlink_path(artifact.temporary);
      const auto parent = artifact.final.has_parent_path() ?
        artifact.final.parent_path() : std::filesystem::path{"."};
      operations.sync_directory(parent);
    }
  } catch (...) {
    while (published_count > 0) {
      --published_count;
      const auto & final_path = artifacts[published_count].final;
      try {
        operations.unlink_path(final_path);
      } catch (...) {
        // Best effort: preserve the original publication exception.
      }
      try {
        const auto parent = final_path.has_parent_path() ?
          final_path.parent_path() : std::filesystem::path{"."};
        operations.sync_directory(parent);
      } catch (...) {
        // Best effort: preserve the original publication exception.
      }
    }
    throw;
  }
}

std::string yaml_quote(const std::string & value)
{
  std::string result;
  if (value.size() > result.max_size() - 2U) {
    throw std::length_error("YAML image filename is too long");
  }
  result.reserve(value.size() + 2U);
  result.push_back('\'');
  for (const unsigned char character : value) {
    if (character < 0x20U || character == 0x7fU) {
      throw std::invalid_argument("YAML image filename contains a control character");
    }
    if (character == '\'') {
      result.push_back('\'');
    }
    result.push_back(static_cast<char>(character));
  }
  result.push_back('\'');
  return result;
}

ProjectionBounds z_bounds(const double sensor_height_m)
{
  if (!std::isfinite(sensor_height_m) || sensor_height_m <= 0.0) {
    throw std::invalid_argument("sensor height must be finite and greater than zero");
  }
  return ProjectionBounds{-sensor_height_m + 0.15, -sensor_height_m + 1.80};
}

Grid project_occupied(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const double resolution,
  const ProjectionBounds bounds)
{
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    throw std::invalid_argument("grid resolution must be finite and greater than zero");
  }
  if (!std::isfinite(bounds.min_z) || !std::isfinite(bounds.max_z) ||
    bounds.min_z > bounds.max_z)
  {
    throw std::invalid_argument("projection z bounds must be finite and ordered");
  }

  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  std::size_t valid_points = 0;
  for (const auto & point : cloud.points) {
    if (!finite_point(point)) {
      continue;
    }
    min_x = std::min(min_x, static_cast<double>(point.x));
    max_x = std::max(max_x, static_cast<double>(point.x));
    min_y = std::min(min_y, static_cast<double>(point.y));
    max_y = std::max(max_y, static_cast<double>(point.y));
    ++valid_points;
  }
  if (valid_points == 0) {
    throw std::invalid_argument("point cloud has no finite XYZ points");
  }

  const double origin_x = min_x - kPaddingMeters;
  const double origin_y = min_y - kPaddingMeters;
  const double extent_x = (max_x - min_x) + 2.0 * kPaddingMeters;
  const double extent_y = (max_y - min_y) + 2.0 * kPaddingMeters;
  if (!std::isfinite(origin_x) || !std::isfinite(origin_y) ||
    !std::isfinite(extent_x) || !std::isfinite(extent_y))
  {
    throw std::length_error("projected grid bounds overflow");
  }

  const std::size_t width = checked_dimension(extent_x, resolution);
  const std::size_t height = checked_dimension(extent_y, resolution);
  if (height > std::numeric_limits<std::size_t>::max() / width) {
    throw std::length_error("projected grid cell count overflows size_t");
  }
  const std::size_t cell_count = width * height;
  if (cell_count > kMaximumGridCells) {
    throw std::length_error("projected grid exceeds the 100 million cell safety limit");
  }

  Grid result{
    width, height, resolution, origin_x, origin_y,
    std::vector<std::uint8_t>(cell_count, kUnknown)};
  for (const auto & point : cloud.points) {
    if (!finite_point(point) || point.z < bounds.min_z || point.z > bounds.max_z) {
      continue;
    }
    const double column_value = std::floor((static_cast<double>(point.x) - origin_x) / resolution);
    const double row_value = std::floor((static_cast<double>(point.y) - origin_y) / resolution);
    if (!std::isfinite(column_value) || !std::isfinite(row_value) ||
      column_value < 0.0 || row_value < 0.0 ||
      column_value >= static_cast<double>(width) ||
      row_value >= static_cast<double>(height))
    {
      continue;
    }
    const auto column = static_cast<std::size_t>(column_value);
    const auto row = static_cast<std::size_t>(row_value);
    result.cells[row * width + column] = kOccupied;
  }
  return result;
}

}  // namespace go1_mapping
