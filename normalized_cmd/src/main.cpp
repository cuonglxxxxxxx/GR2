#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

// Phai khop voi esp32_to_ros_firmware/main/config.h (V_MAX, W_MAX)
#define V_MAX 0.627f
#define W_MAX 5.016f

using namespace std::chrono_literals;

// 2-channel command mux + normalizer:
//   - /cmd_vel        : nav2 (timeout 500ms → force 0 khi Nav2 ngung publish)
//   - /cmd_vel_teleop : teleop (latch vinh vien — bam 1 lan di mai cho den khi bam dung)
//   - Nav2 fresh → override teleop + reset teleop cache (de sau khi Nav2 done thi robot dung)
//   - Output /cmd_vel_normalized (unitless [-1,1]) @ 20Hz
class CmdVelNormalizer : public rclcpp::Node {
public:
  CmdVelNormalizer()
  : Node("cmd_vel_normalizer"),
    has_nav_(false), has_teleop_(false),
    last_nav_time_(0, 0, RCL_ROS_TIME)
  {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_normalized", 10);

    sub_nav_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_nav_ = *msg;
        last_nav_time_ = this->now();
        has_nav_ = true;
      });

    sub_teleop_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_teleop", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_teleop_ = *msg;
        has_teleop_ = true;
      });

    timer_ = this->create_wall_timer(50ms, std::bind(&CmdVelNormalizer::tick, this));

    RCLCPP_INFO(get_logger(),
      "cmd_vel_normalizer: 2-ch mux (Nav2 timeout=500ms, teleop latch) → /cmd_vel_normalized @20Hz");
  }

private:
  void tick() {
    geometry_msgs::msg::Twist cmd;  // default (0,0,0)

    const auto now = this->now();
    const bool nav_fresh = has_nav_ && ((now - last_nav_time_).seconds() < 0.5);

    if (nav_fresh) {
      cmd = last_nav_;
      // Khi Nav2 active, clear teleop cache de sau khi Nav2 ngung
      // thi fallback se la (0,0,0) → robot dung dung lap tuc
      last_teleop_ = geometry_msgs::msg::Twist();
    } else if (has_teleop_) {
      cmd = last_teleop_;
    }
    // else: chua nhan tu ai → giu (0,0,0)

    geometry_msgs::msg::Twist norm;
    norm.linear.x  = cmd.linear.x  / V_MAX;
    norm.angular.z = cmd.angular.z / W_MAX;

    // Clamp [-1, 1] (firmware co clamp lai nhung lam som de log hop ly)
    if (norm.linear.x  >  1.0) norm.linear.x  =  1.0;
    if (norm.linear.x  < -1.0) norm.linear.x  = -1.0;
    if (norm.angular.z >  1.0) norm.angular.z =  1.0;
    if (norm.angular.z < -1.0) norm.angular.z = -1.0;

    pub_->publish(norm);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nav_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_teleop_;
  rclcpp::TimerBase::SharedPtr                               timer_;

  geometry_msgs::msg::Twist last_nav_, last_teleop_;
  bool has_nav_, has_teleop_;
  rclcpp::Time last_nav_time_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelNormalizer>());
  rclcpp::shutdown();
  return 0;
}
