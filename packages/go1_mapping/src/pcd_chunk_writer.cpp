#include "go1_mapping/pcd_chunk_buffer.hpp"

#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace go1_mapping
{
namespace
{

namespace fs = std::filesystem;

std::size_t positive_size_parameter(const int64_t value, const char * const name)
{
  if (value <= 0) {
    throw std::invalid_argument(std::string(name) + " must be greater than zero");
  }
  if (static_cast<uint64_t>(value) > std::numeric_limits<std::size_t>::max()) {
    throw std::invalid_argument(std::string(name) + " does not fit in size_t");
  }
  return static_cast<std::size_t>(value);
}

bool is_strict_descendant(const fs::path & root, const fs::path & candidate)
{
  auto root_component = root.begin();
  auto candidate_component = candidate.begin();
  for (; root_component != root.end(); ++root_component, ++candidate_component) {
    if (candidate_component == candidate.end() || *root_component != *candidate_component) {
      return false;
    }
  }
  return candidate_component != candidate.end();
}

fs::path prepare_output_directory(
  const std::string & allowed_root_parameter,
  const std::string & output_dir_parameter)
{
  if (allowed_root_parameter.empty()) {
    throw std::invalid_argument("allowed_root must not be empty");
  }
  if (output_dir_parameter.empty()) {
    throw std::invalid_argument("output_dir must not be empty");
  }

  const fs::path allowed_root = fs::canonical(fs::path(allowed_root_parameter));
  if (!fs::is_directory(allowed_root)) {
    throw std::invalid_argument("allowed_root is not a directory: " + allowed_root.string());
  }

  const fs::path proposed_output = fs::weakly_canonical(fs::path(output_dir_parameter));
  if (!is_strict_descendant(allowed_root, proposed_output)) {
    throw std::invalid_argument(
            "output_dir must be below allowed_root by canonical path components");
  }

  fs::create_directories(proposed_output);
  const fs::path output_dir = fs::canonical(proposed_output);
  if (!is_strict_descendant(allowed_root, output_dir)) {
    throw std::runtime_error("output_dir escaped allowed_root while it was created");
  }
  return output_dir;
}

std::string chunk_stem(const std::size_t index)
{
  std::ostringstream name;
  name << "chunk_" << std::setfill('0') << std::setw(6) << index;
  return name.str();
}

}  // namespace

class PcdChunkWriter : public rclcpp::Node
{
public:
  PcdChunkWriter()
  : Node("pcd_chunk_writer"),
    output_dir_(prepare_output_directory(
        declare_parameter<std::string>("allowed_root", "/mnt/t500/maps/hanyang_9f"),
        declare_parameter<std::string>("output_dir", ""))),
    buffer_(
      positive_size_parameter(
        declare_parameter<int64_t>("frames_per_chunk", 300), "frames_per_chunk"),
      positive_size_parameter(
        declare_parameter<int64_t>("max_buffer_bytes", 268435456), "max_buffer_bytes")),
    next_chunk_index_(find_next_chunk_index())
  {
    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", qos,
      std::bind(&PcdChunkWriter::on_cloud, this, std::placeholders::_1));
    flush_service_ = create_service<std_srvs::srv::Trigger>(
      "/pcd_chunk_writer/flush",
      std::bind(
        &PcdChunkWriter::on_flush, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Writing PCD chunks below %s", output_dir_.c_str());
  }

private:
  std::size_t find_next_chunk_index() const
  {
    for (std::size_t index = 0; index < std::numeric_limits<std::size_t>::max(); ++index) {
      const auto stem = chunk_stem(index);
      if (!fs::exists(output_dir_ / (stem + ".pcd")) &&
        !fs::exists(output_dir_ / (stem + ".pcd.partial")))
      {
        return index;
      }
    }
    throw std::overflow_error("no PCD chunk index is available");
  }

  void flush_chunk()
  {
    auto cloud = buffer_.take();
    if (cloud->empty()) {
      return;
    }

    const auto stem = chunk_stem(next_chunk_index_);
    const fs::path partial_path = output_dir_ / (stem + ".pcd.partial");
    const fs::path final_path = output_dir_ / (stem + ".pcd");

    if (pcl::io::savePCDFileBinaryCompressed(partial_path.string(), *cloud) < 0) {
      throw std::runtime_error("failed to write partial PCD: " + partial_path.string());
    }
    fs::rename(partial_path, final_path);

    RCLCPP_INFO(
      get_logger(), "Saved %s with %zu points", final_path.c_str(), cloud->size());
    if (next_chunk_index_ == std::numeric_limits<std::size_t>::max()) {
      throw std::overflow_error("PCD chunk index overflow");
    }
    ++next_chunk_index_;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    try {
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::fromROSMsg(*message, cloud);

      if (buffer_.should_flush_before(cloud.size())) {
        flush_chunk();
      }
      buffer_.append(cloud);
    } catch (const std::exception & error) {
      RCLCPP_FATAL(get_logger(), "PCD chunk writer failed: %s", error.what());
      throw;
    }
  }

  void on_flush(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    try {
      flush_chunk();
      response->success = true;
      response->message = "PCD chunk buffer flushed";
    } catch (const std::exception & error) {
      RCLCPP_FATAL(get_logger(), "PCD chunk flush failed: %s", error.what());
      throw;
    }
  }

  const fs::path output_dir_;
  ChunkBuffer buffer_;
  std::size_t next_chunk_index_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr flush_service_;
};

}  // namespace go1_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<go1_mapping::PcdChunkWriter>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "pcd_chunk_writer: " << error.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return EXIT_FAILURE;
  }
}
