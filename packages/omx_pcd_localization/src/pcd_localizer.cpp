#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace omx_pcd_localization
{

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

Eigen::Matrix4f pose_to_matrix(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaternionf q(
    static_cast<float>(pose.orientation.w),
    static_cast<float>(pose.orientation.x),
    static_cast<float>(pose.orientation.y),
    static_cast<float>(pose.orientation.z));
  if (q.norm() < 1.0e-6F) {
    q = Eigen::Quaternionf::Identity();
  } else {
    q.normalize();
  }

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = q.toRotationMatrix();
  transform(0, 3) = static_cast<float>(pose.position.x);
  transform(1, 3) = static_cast<float>(pose.position.y);
  transform(2, 3) = static_cast<float>(pose.position.z);
  return transform;
}

Eigen::Matrix4f transform_to_matrix(const geometry_msgs::msg::Transform & transform_msg)
{
  Eigen::Quaternionf q(
    static_cast<float>(transform_msg.rotation.w),
    static_cast<float>(transform_msg.rotation.x),
    static_cast<float>(transform_msg.rotation.y),
    static_cast<float>(transform_msg.rotation.z));
  if (q.norm() < 1.0e-6F) {
    q = Eigen::Quaternionf::Identity();
  } else {
    q.normalize();
  }

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = q.toRotationMatrix();
  transform(0, 3) = static_cast<float>(transform_msg.translation.x);
  transform(1, 3) = static_cast<float>(transform_msg.translation.y);
  transform(2, 3) = static_cast<float>(transform_msg.translation.z);
  return transform;
}

Eigen::Matrix4f xyz_rpy_to_matrix(const std::vector<double> & values)
{
  const Eigen::AngleAxisf roll(static_cast<float>(values.at(3)), Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf pitch(static_cast<float>(values.at(4)), Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf yaw(static_cast<float>(values.at(5)), Eigen::Vector3f::UnitZ());

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = (yaw * pitch * roll).toRotationMatrix();
  transform(0, 3) = static_cast<float>(values.at(0));
  transform(1, 3) = static_cast<float>(values.at(1));
  transform(2, 3) = static_cast<float>(values.at(2));
  return transform;
}

geometry_msgs::msg::Pose matrix_to_pose(const Eigen::Matrix4f & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform(0, 3);
  pose.position.y = transform(1, 3);
  pose.position.z = transform(2, 3);

  Eigen::Quaternionf q(transform.block<3, 3>(0, 0));
  q.normalize();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

geometry_msgs::msg::Transform matrix_to_transform(const Eigen::Matrix4f & transform)
{
  geometry_msgs::msg::Transform transform_msg;
  transform_msg.translation.x = transform(0, 3);
  transform_msg.translation.y = transform(1, 3);
  transform_msg.translation.z = transform(2, 3);

  Eigen::Quaternionf q(transform.block<3, 3>(0, 0));
  q.normalize();
  transform_msg.rotation.x = q.x();
  transform_msg.rotation.y = q.y();
  transform_msg.rotation.z = q.z();
  transform_msg.rotation.w = q.w();
  return transform_msg;
}

double rotation_distance(const Eigen::Matrix4f & lhs, const Eigen::Matrix4f & rhs)
{
  const Eigen::Matrix3f delta =
    lhs.block<3, 3>(0, 0).transpose() * rhs.block<3, 3>(0, 0);
  const double cosine = std::clamp(
    (static_cast<double>(delta.trace()) - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(cosine);
}

std::pair<double, double> roll_pitch(const Eigen::Matrix4f & transform)
{
  const Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double pitch = std::asin(std::clamp(-static_cast<double>(rotation(2, 0)), -1.0, 1.0));
  return {roll, pitch};
}

bool matrix_is_finite(const Eigen::Matrix4f & transform)
{
  return transform.array().isFinite().all();
}

class PcdLocalizer : public rclcpp::Node
{
public:
  PcdLocalizer()
  : Node("pcd_localizer"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    map_path_ = declare_parameter<std::string>("map_path", "");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered_body");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
    base_frame_ = declare_parameter<std::string>("base_frame", "body");
    map_leaf_size_ = declare_parameter<double>("map_leaf_size", 0.35);
    scan_leaf_size_ = declare_parameter<double>("scan_leaf_size", 0.20);
    local_map_radius_ = declare_parameter<double>("local_map_radius", 18.0);
    local_map_z_radius_ = declare_parameter<double>("local_map_z_radius", 4.0);
    registration_rate_hz_ = declare_parameter<double>("registration_rate_hz", 1.0);
    tf_publish_rate_hz_ = declare_parameter<double>("tf_publish_rate_hz", 20.0);
    localization_timeout_sec_ = declare_parameter<double>("localization_timeout_sec", 3.0);
    min_scan_points_ = declare_parameter<int>("min_scan_points", 300);
    min_target_points_ = declare_parameter<int>("min_target_points", 1000);
    ndt_resolution_ = declare_parameter<double>("ndt_resolution", 1.0);
    ndt_step_size_ = declare_parameter<double>("ndt_step_size", 0.15);
    ndt_transformation_epsilon_ =
      declare_parameter<double>("ndt_transformation_epsilon", 0.01);
    ndt_max_iterations_ = declare_parameter<int>("ndt_max_iterations", 35);
    gicp_max_iterations_ = declare_parameter<int>("gicp_max_iterations", 40);
    gicp_max_correspondence_distance_ =
      declare_parameter<double>("gicp_max_correspondence_distance", 1.2);
    gicp_transformation_epsilon_ =
      declare_parameter<double>("gicp_transformation_epsilon", 0.001);
    gicp_rotation_epsilon_ = declare_parameter<double>("gicp_rotation_epsilon", 0.001);
    gicp_correspondence_randomness_ =
      declare_parameter<int>("gicp_correspondence_randomness", 12);
    max_fitness_score_ = declare_parameter<double>("max_fitness_score", 0.30);
    initial_max_translation_correction_ =
      declare_parameter<double>("initial_max_translation_correction", 3.0);
    initial_max_rotation_correction_ =
      declare_parameter<double>("initial_max_rotation_correction", 1.05);
    tracking_max_translation_correction_ =
      declare_parameter<double>("tracking_max_translation_correction", 0.75);
    tracking_max_rotation_correction_ =
      declare_parameter<double>("tracking_max_rotation_correction", 0.35);
    max_abs_z_ = declare_parameter<double>("max_abs_z", 1.0);
    max_abs_roll_pitch_ = declare_parameter<double>("max_abs_roll_pitch", 0.55);
    use_initial_pose_z_ = declare_parameter<bool>("use_initial_pose_z", false);
    initial_pose_z_ = declare_parameter<double>("initial_pose_z", 0.0);
    publish_aligned_cloud_ = declare_parameter<bool>("publish_aligned_cloud", true);
    publish_map_cloud_ = declare_parameter<bool>("publish_map_cloud", true);
    const auto pcd_to_map_values = declare_parameter<std::vector<double>>(
      "pcd_to_map_xyz_rpy", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    validate_parameters(pcd_to_map_values);
    load_map(xyz_rpy_to_matrix(pcd_to_map_values));

    const auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", transient_qos);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/pose", rclcpp::QoS(10));
    aligned_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/aligned_cloud", rclcpp::SensorDataQoS().keep_last(1));
    map_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/map_cloud", transient_qos);

    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(10),
      std::bind(&PcdLocalizer::on_initial_pose, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS().keep_last(2),
      std::bind(&PcdLocalizer::on_cloud, this, std::placeholders::_1));

    const auto tf_period = std::chrono::duration<double>(1.0 / tf_publish_rate_hz_);
    tf_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(tf_period),
      std::bind(&PcdLocalizer::publish_tf, this));

    if (publish_map_cloud_) {
      publish_map();
    }
    publish_status("WAITING_FOR_INITIAL_POSE");
    RCLCPP_INFO(
      get_logger(), "Loaded %zu map points from %s; waiting for /initialpose",
      map_cloud_->size(), map_path_.c_str());
  }

private:
  void validate_parameters(const std::vector<double> & pcd_to_map_values) const
  {
    if (map_path_.empty()) {
      throw std::invalid_argument("map_path must point to a readable PCD file");
    }
    if (pcd_to_map_values.size() != 6U) {
      throw std::invalid_argument("pcd_to_map_xyz_rpy must contain [x,y,z,roll,pitch,yaw]");
    }
    if (map_leaf_size_ <= 0.0 || scan_leaf_size_ <= 0.0) {
      throw std::invalid_argument("voxel leaf sizes must be positive");
    }
    if (registration_rate_hz_ <= 0.0 || tf_publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("registration and TF rates must be positive");
    }
    if (min_scan_points_ < 20 || min_target_points_ < 50) {
      throw std::invalid_argument("minimum point counts are too small for guarded registration");
    }
  }

  void load_map(const Eigen::Matrix4f & pcd_to_map)
  {
    pcl::PointCloud<pcl::PointXYZI> raw_map;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path_, raw_map) < 0) {
      throw std::runtime_error("failed to load PCD map: " + map_path_);
    }

    auto finite_map = std::make_shared<Cloud>();
    finite_map->reserve(raw_map.size());
    for (const auto & point : raw_map) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        finite_map->push_back(Point{point.x, point.y, point.z});
      }
    }
    if (finite_map->size() < static_cast<std::size_t>(min_target_points_)) {
      throw std::runtime_error("PCD map has too few finite points");
    }

    auto downsampled = std::make_shared<Cloud>();
    pcl::VoxelGrid<Point> voxel;
    voxel.setInputCloud(finite_map);
    const float leaf = static_cast<float>(map_leaf_size_);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*downsampled);

    map_cloud_ = std::make_shared<Cloud>();
    pcl::transformPointCloud(*downsampled, *map_cloud_, pcd_to_map);
    if (map_cloud_->size() < static_cast<std::size_t>(min_target_points_)) {
      throw std::runtime_error("downsampled PCD map has too few points");
    }
  }

  void on_initial_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      RCLCPP_ERROR(
        get_logger(), "Rejected initial pose in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
      publish_status("REJECTED_INITIAL_POSE wrong_frame=" + message->header.frame_id);
      return;
    }

    Eigen::Matrix4f initial = pose_to_matrix(message->pose.pose);
    if (!use_initial_pose_z_) {
      initial(2, 3) = static_cast<float>(initial_pose_z_);
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      initial_map_to_base_ = initial;
      initial_pose_pending_ = true;
      localized_ = false;
      ++initial_pose_generation_;
    }
    publish_status("INITIAL_POSE_RECEIVED");
    RCLCPP_INFO(
      get_logger(), "Initial pose received at [%.2f, %.2f, %.2f]; starting 3D alignment",
      initial(0, 3), initial(1, 3), initial(2, 3));
  }

  Cloud::Ptr cloud_in_base(
    const sensor_msgs::msg::PointCloud2 & message, const rclcpp::Time & stamp)
  {
    auto cloud = std::make_shared<Cloud>();
    pcl::fromROSMsg(message, *cloud);

    auto finite = std::make_shared<Cloud>();
    finite->reserve(cloud->size());
    for (const auto & point : *cloud) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        finite->push_back(point);
      }
    }

    const std::string cloud_frame = message.header.frame_id;
    if (!cloud_frame.empty() && cloud_frame != base_frame_) {
      const auto base_to_cloud_msg = tf_buffer_->lookupTransform(
        base_frame_, cloud_frame, stamp, rclcpp::Duration::from_seconds(0.10));
      const Eigen::Matrix4f base_to_cloud = transform_to_matrix(base_to_cloud_msg.transform);
      auto transformed = std::make_shared<Cloud>();
      pcl::transformPointCloud(*finite, *transformed, base_to_cloud);
      finite = transformed;
    }

    auto filtered = std::make_shared<Cloud>();
    pcl::VoxelGrid<Point> voxel;
    voxel.setInputCloud(finite);
    const float leaf = static_cast<float>(scan_leaf_size_);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*filtered);
    return filtered;
  }

  Cloud::Ptr crop_target(const Eigen::Matrix4f & map_to_base_guess) const
  {
    auto target = std::make_shared<Cloud>();
    pcl::CropBox<Point> crop;
    crop.setInputCloud(map_cloud_);
    const float x = map_to_base_guess(0, 3);
    const float y = map_to_base_guess(1, 3);
    const float z = map_to_base_guess(2, 3);
    crop.setMin(Eigen::Vector4f(
      x - static_cast<float>(local_map_radius_),
      y - static_cast<float>(local_map_radius_),
      z - static_cast<float>(local_map_z_radius_), 1.0F));
    crop.setMax(Eigen::Vector4f(
      x + static_cast<float>(local_map_radius_),
      y + static_cast<float>(local_map_radius_),
      z + static_cast<float>(local_map_z_radius_), 1.0F));
    crop.filter(*target);
    return target;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    const auto callback_time = now();
    if (last_registration_attempt_.nanoseconds() != 0) {
      const double elapsed = (callback_time - last_registration_attempt_).seconds();
      if (elapsed < 1.0 / registration_rate_hz_) {
        return;
      }
    }

    Eigen::Matrix4f map_to_base_guess = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f previous_map_to_odom = Eigen::Matrix4f::Identity();
    bool initial_attempt = false;
    bool localized_snapshot = false;
    std::uint64_t pose_generation = 0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!initial_pose_pending_ && !localized_) {
        return;
      }
      initial_attempt = initial_pose_pending_;
      localized_snapshot = localized_;
      pose_generation = initial_pose_generation_;
      if (initial_attempt) {
        map_to_base_guess = initial_map_to_base_;
      } else {
        previous_map_to_odom = map_to_odom_;
      }
    }
    last_registration_attempt_ = callback_time;

    const rclcpp::Time stamp(message->header.stamp);
    try {
      const auto odom_to_base_msg = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, stamp, rclcpp::Duration::from_seconds(0.10));
      const Eigen::Matrix4f odom_to_base = transform_to_matrix(odom_to_base_msg.transform);
      if (!initial_attempt && localized_snapshot) {
        map_to_base_guess = previous_map_to_odom * odom_to_base;
      }

      auto source = cloud_in_base(*message, stamp);
      if (source->size() < static_cast<std::size_t>(min_scan_points_)) {
        reject("too_few_scan_points=" + std::to_string(source->size()));
        return;
      }
      auto target = crop_target(map_to_base_guess);
      if (target->size() < static_cast<std::size_t>(min_target_points_)) {
        reject("too_few_target_points=" + std::to_string(target->size()));
        return;
      }

      publish_status("ALIGNING");
      pcl::NormalDistributionsTransform<Point, Point> ndt;
      ndt.setInputSource(source);
      ndt.setInputTarget(target);
      ndt.setResolution(static_cast<float>(ndt_resolution_));
      ndt.setStepSize(ndt_step_size_);
      ndt.setTransformationEpsilon(ndt_transformation_epsilon_);
      ndt.setMaximumIterations(ndt_max_iterations_);
      Cloud ndt_output;
      ndt.align(ndt_output, map_to_base_guess);
      if (!ndt.hasConverged() || !matrix_is_finite(ndt.getFinalTransformation())) {
        reject("ndt_not_converged");
        return;
      }

      pcl::GeneralizedIterativeClosestPoint<Point, Point> gicp;
      gicp.setInputSource(source);
      gicp.setInputTarget(target);
      gicp.setMaximumIterations(gicp_max_iterations_);
      gicp.setMaxCorrespondenceDistance(gicp_max_correspondence_distance_);
      gicp.setTransformationEpsilon(gicp_transformation_epsilon_);
      gicp.setRotationEpsilon(gicp_rotation_epsilon_);
      gicp.setCorrespondenceRandomness(gicp_correspondence_randomness_);
      Cloud aligned;
      gicp.align(aligned, ndt.getFinalTransformation());

      const Eigen::Matrix4f refined = gicp.getFinalTransformation();
      const double fitness = gicp.getFitnessScore(gicp_max_correspondence_distance_);
      if (!gicp.hasConverged() || !matrix_is_finite(refined) || !std::isfinite(fitness)) {
        reject("gicp_not_converged");
        return;
      }
      if (fitness > max_fitness_score_) {
        reject("fitness=" + format_number(fitness));
        return;
      }

      const double translation_correction =
        (refined.block<3, 1>(0, 3) - map_to_base_guess.block<3, 1>(0, 3)).norm();
      const double rotation_correction = rotation_distance(map_to_base_guess, refined);
      const double max_translation = initial_attempt ?
        initial_max_translation_correction_ : tracking_max_translation_correction_;
      const double max_rotation = initial_attempt ?
        initial_max_rotation_correction_ : tracking_max_rotation_correction_;
      if (translation_correction > max_translation || rotation_correction > max_rotation) {
        reject(
          "pose_jump translation=" + format_number(translation_correction) +
          " rotation=" + format_number(rotation_correction));
        return;
      }

      const auto [roll, pitch] = roll_pitch(refined);
      if (std::abs(refined(2, 3)) > max_abs_z_ ||
        std::abs(roll) > max_abs_roll_pitch_ ||
        std::abs(pitch) > max_abs_roll_pitch_)
      {
        reject(
          "non_ground_pose z=" + format_number(refined(2, 3)) +
          " roll=" + format_number(roll) + " pitch=" + format_number(pitch));
        return;
      }

      const Eigen::Matrix4f map_to_odom = refined * odom_to_base.inverse();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (pose_generation != initial_pose_generation_) {
          reject("superseded_by_new_initial_pose");
          return;
        }
        map_to_odom_ = map_to_odom;
        initial_pose_pending_ = false;
        localized_ = true;
        last_success_time_ = callback_time;
        last_fitness_ = fitness;
      }

      publish_pose(refined, message->header.stamp, fitness);
      if (publish_aligned_cloud_) {
        publish_aligned_cloud(aligned, message->header.stamp);
      }
      publish_status("LOCALIZED fitness=" + format_number(fitness));
      RCLCPP_INFO(
        get_logger(),
        "Accepted 3D alignment: fitness=%.4f correction=%.3fm/%.2fdeg source=%zu target=%zu",
        fitness, translation_correction, rotation_correction * 180.0 / M_PI,
        source->size(), target->size());
    } catch (const tf2::TransformException & error) {
      reject(std::string("tf_error=") + error.what());
    } catch (const std::exception & error) {
      reject(std::string("registration_error=") + error.what());
    }
  }

  void publish_tf()
  {
    Eigen::Matrix4f transform;
    rclcpp::Time success_time(0, 0, get_clock()->get_clock_type());
    bool localized = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      localized = localized_;
      transform = map_to_odom_;
      success_time = last_success_time_;
    }
    if (!localized) {
      return;
    }

    const auto current_time = now();
    if ((current_time - success_time).seconds() > localization_timeout_sec_) {
      if (!stale_status_published_) {
        publish_status("STALE no_accepted_alignment");
        RCLCPP_ERROR(
          get_logger(), "3D localization is stale; map->%s TF publication stopped",
          odom_frame_.c_str());
        stale_status_published_ = true;
      }
      return;
    }
    stale_status_published_ = false;

    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = current_time;
    message.header.frame_id = map_frame_;
    message.child_frame_id = odom_frame_;
    message.transform = matrix_to_transform(transform);
    tf_broadcaster_->sendTransform(message);
  }

  void publish_pose(
    const Eigen::Matrix4f & map_to_base,
    const builtin_interfaces::msg::Time & stamp,
    double fitness)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = map_frame_;
    message.pose.pose = matrix_to_pose(map_to_base);
    std::fill(message.pose.covariance.begin(), message.pose.covariance.end(), 0.0);
    const double position_variance = std::clamp(fitness, 0.01, 1.0);
    const double rotation_variance = std::clamp(fitness * 0.5, 0.005, 0.5);
    message.pose.covariance[0] = position_variance;
    message.pose.covariance[7] = position_variance;
    message.pose.covariance[14] = position_variance;
    message.pose.covariance[21] = rotation_variance;
    message.pose.covariance[28] = rotation_variance;
    message.pose.covariance[35] = rotation_variance;
    pose_pub_->publish(message);
  }

  void publish_aligned_cloud(
    const Cloud & aligned, const builtin_interfaces::msg::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(aligned, message);
    message.header.stamp = stamp;
    message.header.frame_id = map_frame_;
    aligned_cloud_pub_->publish(message);
  }

  void publish_map()
  {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*map_cloud_, message);
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    map_cloud_pub_->publish(message);
  }

  void publish_status(const std::string & status)
  {
    std_msgs::msg::String message;
    message.data = status;
    if (status_pub_) {
      status_pub_->publish(message);
    }
  }

  void reject(const std::string & reason)
  {
    publish_status("REJECTED " + reason);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Rejected 3D alignment: %s", reason.c_str());
  }

  static std::string format_number(double value)
  {
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(4);
    stream << value;
    return stream.str();
  }

  std::string map_path_;
  std::string cloud_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double map_leaf_size_{};
  double scan_leaf_size_{};
  double local_map_radius_{};
  double local_map_z_radius_{};
  double registration_rate_hz_{};
  double tf_publish_rate_hz_{};
  double localization_timeout_sec_{};
  int min_scan_points_{};
  int min_target_points_{};
  double ndt_resolution_{};
  double ndt_step_size_{};
  double ndt_transformation_epsilon_{};
  int ndt_max_iterations_{};
  int gicp_max_iterations_{};
  double gicp_max_correspondence_distance_{};
  double gicp_transformation_epsilon_{};
  double gicp_rotation_epsilon_{};
  int gicp_correspondence_randomness_{};
  double max_fitness_score_{};
  double initial_max_translation_correction_{};
  double initial_max_rotation_correction_{};
  double tracking_max_translation_correction_{};
  double tracking_max_rotation_correction_{};
  double max_abs_z_{};
  double max_abs_roll_pitch_{};
  bool use_initial_pose_z_{};
  double initial_pose_z_{};
  bool publish_aligned_cloud_{};
  bool publish_map_cloud_{};

  Cloud::Ptr map_cloud_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  std::mutex state_mutex_;
  Eigen::Matrix4f initial_map_to_base_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f map_to_odom_{Eigen::Matrix4f::Identity()};
  bool initial_pose_pending_{false};
  bool localized_{false};
  bool stale_status_published_{false};
  std::uint64_t initial_pose_generation_{0};
  double last_fitness_{std::numeric_limits<double>::infinity()};
  rclcpp::Time last_registration_attempt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_success_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace omx_pcd_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<omx_pcd_localization::PcdLocalizer>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("pcd_localizer"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
