#include "go1_mapping/rate_window.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace go1_mapping {

class TopicRateMonitor : public rclcpp::Node {
 public:
  TopicRateMonitor() : Node("mapping_topic_rate_monitor") {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(256)).best_effort().durability_volatile();
    lidar_subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "/livox/lidar", qos,
        [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr) { lidar_.observe(); });
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        "/livox/imu", qos,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr) { imu_.observe(); });
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr) { odom_.observe(); });
    publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/mapping/input_health", rclcpp::QoS(10).reliable());
    timer_ = create_wall_timer(
        std::chrono::milliseconds(250), [this]() { publish_measurements(); });
  }

 private:
  static diagnostic_msgs::msg::KeyValue value(
      const std::string &key, double number) {
    diagnostic_msgs::msg::KeyValue result;
    result.key = key;
    result.value = std::to_string(number);
    return result;
  }

  void publish_measurements() {
    const auto now = RateWindow::Clock::now();
    const auto lidar = lidar_.measure(now);
    const auto imu = imu_.measure(now);
    const auto odom = odom_.measure(now);
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = "mapping_input_rates";
    status.message = "native receipt-rate measurements";
    status.values = {
        value("lidar_rate_hz", lidar.rate_hz),
        value("imu_rate_hz", imu.rate_hz),
        value("odom_rate_hz", odom.rate_hz),
        value("lidar_gap_sec", lidar.gap_sec),
        value("imu_gap_sec", imu.gap_sec),
        value("odom_gap_sec", odom.gap_sec),
    };
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now_ros();
    message.status.push_back(std::move(status));
    publisher_->publish(message);
  }

  rclcpp::Time now_ros() { return get_clock()->now(); }

  RateWindow lidar_;
  RateWindow imu_;
  RateWindow odom_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace go1_mapping

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go1_mapping::TopicRateMonitor>());
  rclcpp::shutdown();
  return 0;
}