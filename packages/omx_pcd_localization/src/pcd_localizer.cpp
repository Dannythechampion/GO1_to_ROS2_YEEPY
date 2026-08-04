// Guarded 6DoF PCD localization for Go1 FAST-LIO odometry.
//
// Review fixes applied on top of the original nav2-workflow_3D version:
//   P0-1  registration no longer blocks the map -> odom TF timer
//         (separate callback groups + MultiThreadedExecutor)
//   P0-1  TF stamps carry a configurable tolerance, like AMCL
//   P0-2  poses outside the 2D Nav2 map bounds are rejected
//   P1-2  the cropped target, its KD-tree and the NDT/GICP target
//         structures are cached and only rebuilt when the robot moves
//   P1-3  the live scan is cropped to a range the local map can support
//   P1-5  inlier ratio + geometric degeneracy margin are measured,
//         published and (optionally) enforced
//   P2-2  a FAST-LIO restart / odometry origin reset is detected

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
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
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

struct ResidualStats
{
  double mean_squared;
  double inlier_ratio;
  std::size_t used;
};

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

// Mean squared point-to-point residual (capped) plus the inlier ratio.
// The cap keeps a handful of unmatched points from dominating the score.
ResidualStats residual_stats(
  const Cloud & source,
  const Eigen::Matrix4f & transform,
  const pcl::search::KdTree<Point> & tree,
  double cap_distance,
  double inlier_distance)
{
  Cloud moved;
  pcl::transformPointCloud(source, moved, transform);

  const double cap_squared = cap_distance * cap_distance;
  const double inlier_squared = inlier_distance * inlier_distance;

  pcl::Indices indices(1);
  std::vector<float> squared(1);
  double total = 0.0;
  std::size_t used = 0U;
  std::size_t inliers = 0U;

  for (const auto & point : moved) {
    if (tree.nearestKSearch(point, 1, indices, squared) > 0) {
      const double distance_squared = static_cast<double>(squared[0]);
      total += std::min(distance_squared, cap_squared);
      if (distance_squared < inlier_squared) {
        ++inliers;
      }
      ++used;
    }
  }

  if (used == 0U) {
    return ResidualStats{std::numeric_limits<double>::infinity(), 0.0, 0U};
  }
  return ResidualStats{
    total / static_cast<double>(used),
    static_cast<double>(inliers) / static_cast<double>(used),
    used};
}

// Smallest cost increase obtained by sliding the solution horizontally.
// A symmetric corridor lets the scan slide along its axis for free, so a
// small margin means the global fix is not observable from this geometry.
double degeneracy_margin(
  const Cloud & source,
  const Eigen::Matrix4f & transform,
  const pcl::search::KdTree<Point> & tree,
  double cap_distance,
  double base_mean_squared,
  double probe_distance,
  int probe_directions)
{
  double worst = std::numeric_limits<double>::infinity();
  const int directions = std::max(4, probe_directions);
  for (int index = 0; index < directions; ++index) {
    const double angle = 2.0 * M_PI * static_cast<double>(index) /
      static_cast<double>(directions);
    Eigen::Matrix4f probed = transform;
    probed(0, 3) += static_cast<float>(probe_distance * std::cos(angle));
    probed(1, 3) += static_cast<float>(probe_distance * std::sin(angle));
    const auto shifted = residual_stats(source, probed, tree, cap_distance, cap_distance);
    worst = std::min(worst, shifted.mean_squared - base_mean_squared);
  }
  return worst;
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
    map_leaf_size_ = declare_parameter<double>("map_leaf_size", 0.25);
    scan_leaf_size_ = declare_parameter<double>("scan_leaf_size", 0.25);
    scan_max_range_ = declare_parameter<double>("scan_max_range", 15.0);
    scan_max_abs_z_ = declare_parameter<double>("scan_max_abs_z", 4.0);
    local_map_radius_ = declare_parameter<double>("local_map_radius", 25.0);
    local_map_z_radius_ = declare_parameter<double>("local_map_z_radius", 4.0);
    target_refresh_distance_ = declare_parameter<double>("target_refresh_distance", 3.0);
    registration_rate_hz_ = declare_parameter<double>("registration_rate_hz", 1.0);
    tf_publish_rate_hz_ = declare_parameter<double>("tf_publish_rate_hz", 20.0);
    tf_transform_tolerance_ = declare_parameter<double>("tf_transform_tolerance", 0.20);
    localization_timeout_sec_ = declare_parameter<double>("localization_timeout_sec", 3.0);
    min_scan_points_ = declare_parameter<int>("min_scan_points", 300);
    min_target_points_ = declare_parameter<int>("min_target_points", 1000);
    ndt_resolution_ = declare_parameter<double>("ndt_resolution", 1.5);
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
    inlier_distance_ = declare_parameter<double>("inlier_distance", 0.20);
    min_inlier_ratio_ = declare_parameter<double>("min_inlier_ratio", 0.0);
    degeneracy_guard_enabled_ =
      declare_parameter<bool>("degeneracy_guard_enabled", false);
    degeneracy_probe_distance_ =
      declare_parameter<double>("degeneracy_probe_distance", 0.30);
    degeneracy_probe_directions_ =
      declare_parameter<int>("degeneracy_probe_directions", 8);
    min_degeneracy_margin_ = declare_parameter<double>("min_degeneracy_margin", 0.010);
    odom_jump_speed_limit_ = declare_parameter<double>("odom_jump_speed_limit", 3.0);
    use_initial_pose_z_ = declare_parameter<bool>("use_initial_pose_z", false);
    initial_pose_z_ = declare_parameter<double>("initial_pose_z", 0.0);
    publish_aligned_cloud_ = declare_parameter<bool>("publish_aligned_cloud", true);
    publish_map_cloud_ = declare_parameter<bool>("publish_map_cloud", true);
    map_bounds_xy_ = declare_parameter<std::vector<double>>(
      "map_bounds_xy", std::vector<double>{});
    const auto pcd_to_map_values = declare_parameter<std::vector<double>>(
      "pcd_to_map_xyz_rpy", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    validate_parameters(pcd_to_map_values);
    load_map(xyz_rpy_to_matrix(pcd_to_map_values));

    registration_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    output_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    const auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", transient_qos);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/pose", rclcpp::QoS(10));
    aligned_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/aligned_cloud", rclcpp::SensorDataQoS().keep_last(1));
    map_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/map_cloud", transient_qos);

    rclcpp::SubscriptionOptions initial_pose_options;
    initial_pose_options.callback_group = output_group_;
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(10),
      std::bind(&PcdLocalizer::on_initial_pose, this, std::placeholders::_1),
      initial_pose_options);

    rclcpp::SubscriptionOptions cloud_options;
    cloud_options.callback_group = registration_group_;
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS().keep_last(2),
      std::bind(&PcdLocalizer::on_cloud, this, std::placeholders::_1),
      cloud_options);

    const auto tf_period = std::chrono::duration<double>(1.0 / tf_publish_rate_hz_);
    tf_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(tf_period),
      std::bind(&PcdLocalizer::publish_tf, this),
      output_group_);

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
    if (scan_max_range_ >= local_map_radius_) {
      throw std::invalid_argument(
              "scan_max_range must be smaller than local_map_radius so every "
              "live point can be matched against the cropped map");
    }
    if (!map_bounds_xy_.empty() && map_bounds_xy_.size() != 4U) {
      throw std::invalid_argument("map_bounds_xy must be empty or [min_x,min_y,max_x,max_y]");
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

  bool inside_map_bounds(const Eigen::Matrix4f & map_to_base) const
  {
    if (map_bounds_xy_.size() != 4U) {
      return true;
    }
    const double x = static_cast<double>(map_to_base(0, 3));
    const double y = static_cast<double>(map_to_base(1, 3));
    return x >= map_bounds_xy_[0] && y >= map_bounds_xy_[1] &&
           x <= map_bounds_xy_[2] && y <= map_bounds_xy_[3];
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
    if (!inside_map_bounds(initial)) {
      RCLCPP_ERROR(
        get_logger(), "Rejected initial pose outside the 2D map bounds: [%.2f, %.2f]",
        initial(0, 3), initial(1, 3));
      publish_status("REJECTED_INITIAL_POSE outside_map_bounds");
      return;
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

    // Drop points the cropped local map cannot possibly explain. Without this
    // they still enter the GICP cost and inflate every fitness measurement.
    auto cropped = std::make_shared<Cloud>();
    pcl::CropBox<Point> range_crop;
    range_crop.setInputCloud(finite);
    const float range = static_cast<float>(scan_max_range_);
    const float height = static_cast<float>(scan_max_abs_z_);
    range_crop.setMin(Eigen::Vector4f(-range, -range, -height, 1.0F));
    range_crop.setMax(Eigen::Vector4f(range, range, height, 1.0F));
    range_crop.filter(*cropped);

    auto filtered = std::make_shared<Cloud>();
    pcl::VoxelGrid<Point> voxel;
    voxel.setInputCloud(cropped);
    const float leaf = static_cast<float>(scan_leaf_size_);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*filtered);
    return filtered;
  }

  // Returns the cached target, rebuilding the crop, the KD-tree and the NDT /
  // GICP target structures only when the robot has actually moved. Rebuilding
  // GICP target covariances every cycle was the dominant per-cycle CPU cost.
  Cloud::Ptr target_for(const Eigen::Matrix4f & map_to_base_guess, bool force_refresh)
  {
    const Eigen::Vector3f center = map_to_base_guess.block<3, 1>(0, 3);
    if (!force_refresh && cached_target_ &&
      (center - cached_target_center_).norm() < static_cast<float>(target_refresh_distance_))
    {
      return cached_target_;
    }

    auto target = std::make_shared<Cloud>();
    pcl::CropBox<Point> crop;
    crop.setInputCloud(map_cloud_);
    crop.setMin(Eigen::Vector4f(
        center.x() - static_cast<float>(local_map_radius_),
        center.y() - static_cast<float>(local_map_radius_),
        center.z() - static_cast<float>(local_map_z_radius_), 1.0F));
    crop.setMax(Eigen::Vector4f(
        center.x() + static_cast<float>(local_map_radius_),
        center.y() + static_cast<float>(local_map_radius_),
        center.z() + static_cast<float>(local_map_z_radius_), 1.0F));
    crop.filter(*target);

    if (target->size() < static_cast<std::size_t>(min_target_points_)) {
      return target;
    }

    ndt_.setInputTarget(target);
    gicp_.setInputTarget(target);
    target_tree_.setInputCloud(target);

    cached_target_ = target;
    cached_target_center_ = center;
    return cached_target_;
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

      // A FAST-LIO restart redefines camera_init. Detect the discontinuity
      // instead of publishing a confidently wrong map -> odom for 5 seconds.
      if (has_previous_odom_) {
        const double interval = (stamp - previous_odom_stamp_).seconds();
        const double jump =
          (odom_to_base.block<3, 1>(0, 3) - previous_odom_to_base_.block<3, 1>(0, 3)).norm();
        if (interval > 0.0 && jump > odom_jump_speed_limit_ * interval) {
          drop_localization("odom_reset jump=" + format_number(jump));
          previous_odom_to_base_ = odom_to_base;
          previous_odom_stamp_ = stamp;
          return;
        }
      }
      previous_odom_to_base_ = odom_to_base;
      previous_odom_stamp_ = stamp;
      has_previous_odom_ = true;

      if (!initial_attempt && localized_snapshot) {
        map_to_base_guess = previous_map_to_odom * odom_to_base;
      }

      auto source = cloud_in_base(*message, stamp);
      if (source->size() < static_cast<std::size_t>(min_scan_points_)) {
        reject("too_few_scan_points=" + std::to_string(source->size()));
        return;
      }
      auto target = target_for(map_to_base_guess, initial_attempt);
      if (target->size() < static_cast<std::size_t>(min_target_points_)) {
        reject("too_few_target_points=" + std::to_string(target->size()));
        return;
      }

      publish_status("ALIGNING");
      ndt_.setInputSource(source);
      ndt_.setResolution(static_cast<float>(ndt_resolution_));
      ndt_.setStepSize(ndt_step_size_);
      ndt_.setTransformationEpsilon(ndt_transformation_epsilon_);
      ndt_.setMaximumIterations(ndt_max_iterations_);
      Cloud ndt_output;
      ndt_.align(ndt_output, map_to_base_guess);
      if (!ndt_.hasConverged() || !matrix_is_finite(ndt_.getFinalTransformation())) {
        reject("ndt_not_converged");
        return;
      }

      gicp_.setInputSource(source);
      gicp_.setMaximumIterations(gicp_max_iterations_);
      gicp_.setMaxCorrespondenceDistance(gicp_max_correspondence_distance_);
      gicp_.setTransformationEpsilon(gicp_transformation_epsilon_);
      gicp_.setRotationEpsilon(gicp_rotation_epsilon_);
      gicp_.setCorrespondenceRandomness(gicp_correspondence_randomness_);
      Cloud aligned;
      gicp_.align(aligned, ndt_.getFinalTransformation());

      const Eigen::Matrix4f refined = gicp_.getFinalTransformation();
      const double fitness = gicp_.getFitnessScore(gicp_max_correspondence_distance_);
      if (!gicp_.hasConverged() || !matrix_is_finite(refined) || !std::isfinite(fitness)) {
        reject("gicp_not_converged");
        return;
      }
      if (fitness > max_fitness_score_) {
        reject("fitness=" + format_number(fitness));
        return;
      }

      const auto stats = residual_stats(
        *source, refined, target_tree_, gicp_max_correspondence_distance_, inlier_distance_);
      if (min_inlier_ratio_ > 0.0 && stats.inlier_ratio < min_inlier_ratio_) {
        reject("low_inlier_ratio=" + format_number(stats.inlier_ratio));
        return;
      }

      const double margin = degeneracy_margin(
        *source, refined, target_tree_, gicp_max_correspondence_distance_,
        stats.mean_squared, degeneracy_probe_distance_, degeneracy_probe_directions_);
      if (degeneracy_guard_enabled_ && margin < min_degeneracy_margin_) {
        reject("degenerate_geometry margin=" + format_number(margin));
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

      // A 3D pose outside the 2D occupancy map cannot be planned in.
      if (!inside_map_bounds(refined)) {
        reject(
          "outside_map_bounds x=" + format_number(refined(0, 3)) +
          " y=" + format_number(refined(1, 3)));
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
      publish_status(
        "LOCALIZED fitness=" + format_number(fitness) +
        " inlier=" + format_number(stats.inlier_ratio) +
        " margin=" + format_number(margin));
      RCLCPP_INFO(
        get_logger(),
        "Accepted 3D alignment: fitness=%.4f inlier=%.3f margin=%.4f "
        "correction=%.3fm/%.2fdeg source=%zu target=%zu",
        fitness, stats.inlier_ratio, margin, translation_correction,
        rotation_correction * 180.0 / M_PI, source->size(), target->size());
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
    message.header.stamp =
      current_time + rclcpp::Duration::from_seconds(tf_transform_tolerance_);
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

  void drop_localization(const std::string & reason)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      localized_ = false;
      initial_pose_pending_ = false;
    }
    cached_target_.reset();
    publish_status("REJECTED " + reason + " waiting_for_initial_pose");
    RCLCPP_ERROR(
      get_logger(),
      "Odometry discontinuity detected (%s); stopped publishing map->%s. "
      "Send a new 2D Pose Estimate before driving.",
      reason.c_str(), odom_frame_.c_str());
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
  double scan_max_range_{};
  double scan_max_abs_z_{};
  double local_map_radius_{};
  double local_map_z_radius_{};
  double target_refresh_distance_{};
  double registration_rate_hz_{};
  double tf_publish_rate_hz_{};
  double tf_transform_tolerance_{};
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
  double inlier_distance_{};
  double min_inlier_ratio_{};
  bool degeneracy_guard_enabled_{};
  double degeneracy_probe_distance_{};
  int degeneracy_probe_directions_{};
  double min_degeneracy_margin_{};
  double odom_jump_speed_limit_{};
  bool use_initial_pose_z_{};
  double initial_pose_z_{};
  bool publish_aligned_cloud_{};
  bool publish_map_cloud_{};
  std::vector<double> map_bounds_xy_;

  Cloud::Ptr map_cloud_;
  Cloud::Ptr cached_target_;
  Eigen::Vector3f cached_target_center_{Eigen::Vector3f::Zero()};
  pcl::NormalDistributionsTransform<Point, Point> ndt_;
  pcl::GeneralizedIterativeClosestPoint<Point, Point> gicp_;
  pcl::search::KdTree<Point> target_tree_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::CallbackGroup::SharedPtr registration_group_;
  rclcpp::CallbackGroup::SharedPtr output_group_;
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
  Eigen::Matrix4f previous_odom_to_base_{Eigen::Matrix4f::Identity()};
  rclcpp::Time previous_odom_stamp_{0, 0, RCL_ROS_TIME};
  bool has_previous_odom_{false};
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
    auto node = std::make_shared<omx_pcd_localization::PcdLocalizer>();
    // Three threads: registration, TF/initial pose, spare. Without this the
    // NDT/GICP call blocks the 20 Hz map -> odom broadcast and Nav2 trips its
    // 0.3 s transform_tolerance on every cycle.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3U);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("pcd_localizer"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
