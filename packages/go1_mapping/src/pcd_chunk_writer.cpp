#include "go1_mapping/pcd_chunk_buffer.hpp"
#include "go1_mapping/pcd_chunk_storage.hpp"

#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
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

int64_t declare_max_buffer_bytes_parameter(rclcpp::Node & node)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description =
    "Maximum retained uncompressed pcl::PointXYZI payload bytes. The DDS-owned "
    "PointCloud2 message and PCL compression workspace are separate overhead; incoming "
    "serialized data must also fit this limit before conversion.";
  return node.declare_parameter<int64_t>("max_buffer_bytes", 268435456, descriptor);
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

}  // namespace

class PcdChunkWriter : public rclcpp::Node
{
public:
  PcdChunkWriter()
  : Node("pcd_chunk_writer"),
    output_dir_(prepare_output_directory(
        declare_parameter<std::string>("allowed_root", "/mnt/t500/maps/hanyang_9f"),
        declare_parameter<std::string>("output_dir", ""))),
    max_buffer_bytes_(positive_size_parameter(
        declare_max_buffer_bytes_parameter(*this), "max_buffer_bytes")),
    buffer_(
      positive_size_parameter(
        declare_parameter<int64_t>("frames_per_chunk", 300), "frames_per_chunk"),
      max_buffer_bytes_),
    storage_(output_dir_)
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
  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    try {
      const auto validated = validate_cloud_message(*message, max_buffer_bytes_);
      if (buffer_.should_flush_before(validated.point_count)) {
        storage_.flush(buffer_);
      }

      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::fromROSMsg(*message, cloud);
      if (cloud.size() != validated.point_count) {
        throw std::invalid_argument("PCL conversion changed the validated point count");
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
      const bool wrote_file = storage_.flush(buffer_);
      response->success = true;
      response->message = wrote_file ? "PCD chunk persisted" : "PCD chunk buffer was empty";
    } catch (const std::exception & error) {
      RCLCPP_FATAL(get_logger(), "PCD chunk flush failed: %s", error.what());
      throw;
    }
  }

  const fs::path output_dir_;
  const std::size_t max_buffer_bytes_;
  ChunkBuffer buffer_;
  PcdChunkStorage storage_;
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
