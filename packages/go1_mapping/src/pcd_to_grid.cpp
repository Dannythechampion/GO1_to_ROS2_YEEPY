#include "go1_mapping/pcd_projection.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <fcntl.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unistd.h>

namespace
{

namespace fs = std::filesystem;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

struct Options
{
  fs::path input_dir;
  fs::path output_pcd;
  fs::path output_map_prefix;
  double sensor_height_m{0.0};
  double resolution{0.05};
  double voxel_size{0.10};
};

fs::path parent_or_current(const fs::path & path)
{
  return path.has_parent_path() ? path.parent_path() : fs::path{"."};
}

bool is_chunk_name(const fs::path & path)
{
  const std::string name = path.filename().string();
  return name.size() > 10 && name.rfind("chunk_", 0) == 0 && path.extension() == ".pcd";
}

fs::path normalized_path(const fs::path & path)
{
  std::error_code error;
  const fs::path result = fs::weakly_canonical(fs::absolute(path), error);
  if (error) {
    throw std::system_error(error, "cannot normalize path " + path.string());
  }
  return result;
}

double positive_number(const std::string & text, const std::string & option)
{
  std::size_t consumed = 0;
  double result = 0.0;
  try {
    result = std::stod(text, &consumed);
  } catch (const std::exception &) {
    throw std::invalid_argument(option + " must be a number");
  }
  if (consumed != text.size() || !std::isfinite(result) || result <= 0.0) {
    throw std::invalid_argument(option + " must be finite and greater than zero");
  }
  return result;
}

Options parse_options(const int argc, char ** argv)
{
  Options options;
  std::set<std::string> seen;
  for (int index = 1; index < argc; index += 2) {
    const std::string flag = argv[index];
    if (index + 1 >= argc || flag.rfind("--", 0) != 0) {
      throw std::invalid_argument("every option requires a value");
    }
    if (!seen.insert(flag).second) {
      throw std::invalid_argument("duplicate option: " + flag);
    }
    const std::string value = argv[index + 1];
    if (flag == "--input-dir") {
      options.input_dir = value;
    } else if (flag == "--output-pcd") {
      options.output_pcd = value;
    } else if (flag == "--output-map") {
      options.output_map_prefix = value;
    } else if (flag == "--sensor-height-m") {
      options.sensor_height_m = positive_number(value, flag);
    } else if (flag == "--resolution") {
      options.resolution = positive_number(value, flag);
    } else if (flag == "--voxel-size") {
      options.voxel_size = positive_number(value, flag);
    } else {
      throw std::invalid_argument("unknown option: " + flag);
    }
  }
  if (options.input_dir.empty() || options.output_pcd.empty() ||
    options.output_map_prefix.empty() || options.sensor_height_m <= 0.0)
  {
    throw std::invalid_argument(
            "required: --input-dir DIR --output-pcd FILE --output-map PREFIX "
            "--sensor-height-m METERS");
  }
  return options;
}

std::vector<fs::path> find_chunks(const fs::path & input_dir)
{
  if (!fs::is_directory(input_dir)) {
    throw std::invalid_argument("input directory does not exist or is not a directory");
  }
  std::vector<fs::path> chunks;
  for (const auto & entry : fs::directory_iterator(input_dir)) {
    if (entry.is_regular_file() && is_chunk_name(entry.path())) {
      chunks.push_back(entry.path());
    }
  }
  std::sort(chunks.begin(), chunks.end());
  if (chunks.empty()) {
    throw std::invalid_argument("input directory contains no chunk_*.pcd files");
  }
  return chunks;
}

Cloud merge_chunks(const std::vector<fs::path> & chunks, const double voxel_size)
{
  Cloud merged;
  for (const auto & chunk_path : chunks) {
    Cloud chunk;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(chunk_path.string(), chunk) < 0) {
      throw std::runtime_error("failed to read PCD chunk: " + chunk_path.string());
    }
    const Cloud filtered = go1_mapping::voxel_filter_finite(chunk, voxel_size);
    const std::size_t combined_size = go1_mapping::checked_merged_point_count(
      merged.size(), filtered.size());
    if (combined_size > std::numeric_limits<std::uint32_t>::max() ||
      combined_size > merged.points.max_size())
    {
      throw std::length_error("merged point count exceeds PCL limits");
    }
    merged.reserve(combined_size);
    merged += filtered;
  }
  if (merged.empty()) {
    throw std::invalid_argument("all input PCD chunks are empty");
  }
  Cloud final_cloud = go1_mapping::voxel_filter_finite(merged, voxel_size);
  if (final_cloud.empty()) {
    throw std::invalid_argument("voxel filtering produced an empty point cloud");
  }
  return final_cloud;
}

class TemporaryArtifact
{
public:
  explicit TemporaryArtifact(const fs::path & final_path)
  {
    const fs::path directory = parent_or_current(final_path);
    const std::string base = final_path.filename().string();
    for (std::size_t attempt = 0; attempt < 1000; ++attempt) {
      path_ = directory /
        (base + ".partial." + std::to_string(static_cast<long long>(getpid())) + "." +
        std::to_string(attempt));
      const int descriptor = open(
        path_.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0644);
      if (descriptor >= 0) {
        if (close(descriptor) != 0) {
          const int close_error = errno;
          unlink(path_.c_str());
          throw std::system_error(close_error, std::generic_category(), "close temporary artifact");
        }
        return;
      }
      if (errno != EEXIST) {
        throw std::system_error(errno, std::generic_category(), "create temporary artifact");
      }
    }
    throw std::runtime_error("could not allocate a unique temporary artifact name");
  }

  ~TemporaryArtifact()
  {
    unlink(path_.c_str());
  }

  TemporaryArtifact(const TemporaryArtifact &) = delete;
  TemporaryArtifact & operator=(const TemporaryArtifact &) = delete;

  const fs::path & path() const noexcept {return path_;}
private:
  fs::path path_;
};

void sync_file(const fs::path & path)
{
  const int descriptor = open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (descriptor < 0) {
    throw std::system_error(errno, std::generic_category(), "open artifact for fsync");
  }
  if (fsync(descriptor) != 0) {
    const int sync_error = errno;
    close(descriptor);
    throw std::system_error(sync_error, std::generic_category(), "fsync artifact");
  }
  if (close(descriptor) != 0) {
    throw std::system_error(errno, std::generic_category(), "close fsynced artifact");
  }
}

void sync_directory(const fs::path & path)
{
  const int descriptor = open(path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
  if (descriptor < 0) {
    throw std::system_error(errno, std::generic_category(), "open output directory for fsync");
  }
  if (fsync(descriptor) != 0) {
    const int sync_error = errno;
    close(descriptor);
    throw std::system_error(sync_error, std::generic_category(), "fsync output directory");
  }
  if (close(descriptor) != 0) {
    throw std::system_error(errno, std::generic_category(), "close fsynced output directory");
  }
}

void write_pgm(const fs::path & path, const go1_mapping::Grid & grid)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("failed to open temporary PGM");
  }
  output << "P5\n" << grid.width << " " << grid.height << "\n255\n";
  for (std::size_t row = grid.height; row > 0; --row) {
    const std::size_t offset = (row - 1) * grid.width;
    output.write(
      reinterpret_cast<const char *>(grid.cells.data() + offset),
      static_cast<std::streamsize>(grid.width));
  }
  output.close();
  if (!output) {
    throw std::runtime_error("failed to write temporary PGM");
  }
}

void write_yaml(
  const fs::path & path,
  const fs::path & image_path,
  const go1_mapping::Grid & grid)
{
  std::ofstream output(path, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("failed to open temporary map YAML");
  }
  output << "image: " << go1_mapping::yaml_quote(image_path.filename().string()) << "\n"
         << std::setprecision(17)
         << "resolution: " << grid.resolution << "\n"
         << "origin: [" << grid.origin_x << ", " << grid.origin_y << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n"
         << "go1_mapping_role: geometry_reference_only\n";
  output.close();
  if (!output) {
    throw std::runtime_error("failed to write temporary map YAML");
  }
}

void ensure_unused_outputs(const std::vector<fs::path> & outputs)
{
  for (const auto & output : outputs) {
    if (fs::exists(output)) {
      throw std::invalid_argument("refusing to overwrite existing output: " + output.string());
    }
  }
}

int run(const int argc, char ** argv)
{
  const Options options = parse_options(argc, argv);
  const std::vector<fs::path> chunks = find_chunks(options.input_dir);
  const fs::path output_pgm = options.output_map_prefix.string() + ".pgm";
  const fs::path output_yaml = options.output_map_prefix.string() + ".yaml";
  const std::vector<fs::path> outputs{options.output_pcd, output_pgm, output_yaml};

  for (const auto & output : outputs) {
    fs::create_directories(parent_or_current(output));
  }
  ensure_unused_outputs(outputs);
  std::set<fs::path> normalized_outputs;
  for (const auto & output : outputs) {
    if (!normalized_outputs.insert(normalized_path(output)).second) {
      throw std::invalid_argument("output PCD, PGM, and YAML paths must be distinct");
    }
  }

  const fs::path normalized_output_pcd = normalized_path(options.output_pcd);
  const fs::path normalized_input_dir = normalized_path(options.input_dir);
  if (is_chunk_name(options.output_pcd) &&
    normalized_path(parent_or_current(options.output_pcd)) == normalized_input_dir)
  {
    throw std::invalid_argument("output PCD must not match the input chunk naming pattern");
  }
  for (const auto & chunk : chunks) {
    if (normalized_path(chunk) == normalized_output_pcd) {
      throw std::invalid_argument("output PCD must not be an input chunk");
    }
  }

  const Cloud merged = merge_chunks(chunks, options.voxel_size);
  const go1_mapping::Grid grid = go1_mapping::project_occupied(
    merged, options.resolution, go1_mapping::z_bounds(options.sensor_height_m));

  TemporaryArtifact pcd_temporary(options.output_pcd);
  TemporaryArtifact pgm_temporary(output_pgm);
  TemporaryArtifact yaml_temporary(output_yaml);
  if (pcl::io::savePCDFileBinaryCompressed(pcd_temporary.path().string(), merged) < 0) {
    throw std::runtime_error("failed to write binary-compressed merged PCD");
  }
  write_pgm(pgm_temporary.path(), grid);
  write_yaml(yaml_temporary.path(), output_pgm, grid);
  sync_file(pcd_temporary.path());
  sync_file(pgm_temporary.path());
  sync_file(yaml_temporary.path());

  const std::vector<go1_mapping::PublicationArtifact> artifacts{
    {pcd_temporary.path(), options.output_pcd},
    {pgm_temporary.path(), output_pgm},
    {yaml_temporary.path(), output_yaml}};
  go1_mapping::PublicationOperations publication_operations;
  publication_operations.link_no_replace = [](
    const fs::path & temporary, const fs::path & final_path)
    {
      if (link(temporary.c_str(), final_path.c_str()) != 0) {
        throw std::system_error(
                errno, std::generic_category(),
                "publish artifact without overwrite");
      }
    };
  publication_operations.unlink_path = [](const fs::path & path) {
      if (unlink(path.c_str()) != 0) {
        throw std::system_error(errno, std::generic_category(), "unlink artifact");
      }
    };
  publication_operations.sync_directory = [](const fs::path & directory) {
      sync_directory(directory);
    };
  go1_mapping::publish_artifacts(artifacts, publication_operations);

  std::cout << "Merged " << chunks.size() << " chunks into " << merged.size()
            << " filtered points; geometry reference: " << output_yaml << '\n';
  return 0;
}

}  // namespace

int main(const int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << "pcd_to_grid: " << error.what() << '\n';
    return 1;
  }
}
