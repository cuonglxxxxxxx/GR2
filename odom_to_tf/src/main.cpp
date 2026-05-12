#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class OdomSimpleBridge : public rclcpp::Node
{
public:
  OdomSimpleBridge()
  : Node("odom_simple_bridge")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscribe dữ liệu từ robot
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_msg_ = msg;
        has_data_ = true;
      });

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        last_imu_msg_ = msg;
      });

    // Publisher cho dữ liệu sạch để RViz dùng (Re-stamped)
    synced_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_synced", 10);

    // TIMER CỐ ĐỊNH 50HZ: Đảm bảo TF tree không bao giờ bị đứt gãy dù WiFi lag
    timer_ = this->create_wall_timer(20ms, std::bind(&OdomSimpleBridge::publishTf, this));

    RCLCPP_INFO(get_logger(), "OdomSimpleBridge: Fixed 50Hz TF active (Leonardo Logic).");
  }

private:
  void publishTf()
  {
    if (!has_data_) return;

    auto now = this->get_clock()->now();

    // 1. Phát TF odom -> base_footprint (Sử dụng thời gian PC + 50ms buffer)
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";
    t.transform.translation.x = last_odom_msg_->pose.pose.position.x;
    t.transform.translation.y = last_odom_msg_->pose.pose.position.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = last_odom_msg_->pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);

    // 2. Re-stamp IMU để RViz hiển thị mượt mà
    if (last_imu_msg_) {
        sensor_msgs::msg::Imu synced_imu = *last_imu_msg_;
        synced_imu.header.stamp = now;
        synced_imu_pub_->publish(synced_imu);
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr synced_imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  nav_msgs::msg::Odometry::SharedPtr last_odom_msg_;
  sensor_msgs::msg::Imu::SharedPtr last_imu_msg_;
  bool has_data_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomSimpleBridge>());
  rclcpp::shutdown();
  return 0;
}
