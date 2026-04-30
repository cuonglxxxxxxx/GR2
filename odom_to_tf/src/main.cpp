#include <memory>
#include <chrono>

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
      10ms, std::bind(&OdomToTfNode::publishTf, this)
    );

    RCLCPP_INFO(get_logger(), "OdomToTfNode: Direct base_link mode.");
  }

private:
  void publishTf()
  {
    if (!has_data_) return;

    // Freshness check: If odom data is older than 2.0s, stop publishing to avoid "jumping"
    auto now = this->get_clock()->now();
    auto diff = now - last_odom_msg_->header.stamp;
    if (diff.seconds() > 2.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Odom data too old (%.2f s), skipping TF", diff.seconds());
        return;
    }

    geometry_msgs::msg::TransformStamped t;

    // Use current PC time for TF to keep RViz happy.
    t.header.stamp = now;
    
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";

    t.transform.translation.x = last_odom_msg_->pose.pose.position.x;
    t.transform.translation.y = last_odom_msg_->pose.pose.position.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = last_odom_msg_->pose.pose.orientation;

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
