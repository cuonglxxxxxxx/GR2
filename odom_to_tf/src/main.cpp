#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class OdomToTfNode : public rclcpp::Node
{
public:
  OdomToTfNode()
  : Node("odom_to_tf_node")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->last_odom_msg_ = msg;
        this->has_data_ = true;
      }
    );

    timer_ = this->create_wall_timer(
      20ms, std::bind(&OdomToTfNode::publishTf, this) // 50Hz
    );

    RCLCPP_INFO(get_logger(), "OdomToTfNode: Stabilized 50Hz mode with tolerance.");
  }

private:
  void publishTf()
  {
    if (!has_data_) return;

    auto now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped t;

    // Add a small tolerance (0.05s) to the timestamp. 
    // This tells RViz the transform is valid for a short window,
    // preventing the robot from turning white if the next packet is delayed.
    t.header.stamp = now + rclcpp::Duration::from_seconds(0.05);
    
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";

    t.transform.translation.x = last_odom_msg_->pose.pose.position.x;
    t.transform.translation.y = last_odom_msg_->pose.pose.position.y;
    t.transform.translation.z = 0.0;

    t.transform.rotation = last_odom_msg_->pose.pose.orientation;
    
    // Ensure orientation is valid to prevent jumping to infinity
    if (std::abs(t.transform.rotation.w) < 0.001 && std::abs(t.transform.rotation.x) < 0.001 && 
        std::abs(t.transform.rotation.y) < 0.001 && std::abs(t.transform.rotation.z) < 0.001) {
        t.transform.rotation.w = 1.0;
    }

    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_msg_;
  bool has_data_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomToTfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
