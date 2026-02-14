#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

namespace {
constexpr int32_t kApiStopMove = 1003;
constexpr int32_t kApiStandUp = 1004;
constexpr int32_t kApiMove = 1008;
}  // namespace

class Go2wCmdVelControlNode : public rclcpp::Node {
 public:
  Go2wCmdVelControlNode()
      : Node("go2w_cmd_vel_control"),
        latest_cmd_(std::make_shared<geometry_msgs::msg::Twist>()),
        is_stopped_(true) {
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    request_topic_ =
        this->declare_parameter<std::string>("request_topic", "/api/sport/request");
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 20.0);
    cmd_timeout_sec_ = this->declare_parameter<double>("cmd_timeout_sec", 0.5);
    max_linear_x_ = this->declare_parameter<double>("max_linear_x", 0.6);
    max_linear_y_ = this->declare_parameter<double>("max_linear_y", 0.4);
    max_angular_z_ = this->declare_parameter<double>("max_angular_z", 1.0);
    auto_stand_up_ = this->declare_parameter<bool>("auto_stand_up", true);

    request_pub_ = this->create_publisher<unitree_api::msg::Request>(request_topic_, 10);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->OnCmdVel(msg); });

    last_cmd_time_ = this->now();

    if (control_rate_hz_ <= 0.0) {
      control_rate_hz_ = 20.0;
    }
    auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { this->OnTimer(); });

    if (auto_stand_up_) {
      PublishStandUp();
      RCLCPP_INFO(this->get_logger(), "Sent StandUp (api_id=%d)", kApiStandUp);
    }

    RCLCPP_INFO(this->get_logger(),
                "go2w_cmd_vel_control started. cmd_vel=%s request=%s rate=%.1fHz timeout=%.2fs",
                cmd_vel_topic_.c_str(), request_topic_.c_str(), control_rate_hz_,
                cmd_timeout_sec_);
  }

 private:
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    latest_cmd_->linear.x = Clamp(msg->linear.x, -max_linear_x_, max_linear_x_);
    latest_cmd_->linear.y = Clamp(msg->linear.y, -max_linear_y_, max_linear_y_);
    latest_cmd_->angular.z = Clamp(msg->angular.z, -max_angular_z_, max_angular_z_);
    last_cmd_time_ = this->now();
  }

  void OnTimer() {
    const double age = (this->now() - last_cmd_time_).seconds();
    if (age > cmd_timeout_sec_) {
      if (!is_stopped_) {
        PublishStop();
        is_stopped_ = true;
      }
      return;
    }

    PublishMove(latest_cmd_->linear.x, latest_cmd_->linear.y, latest_cmd_->angular.z);
    is_stopped_ = false;
  }

  void PublishStandUp() {
    unitree_api::msg::Request req;
    req.header.identity.api_id = kApiStandUp;
    req.parameter.clear();
    request_pub_->publish(req);
  }

  void PublishStop() {
    unitree_api::msg::Request req;
    req.header.identity.api_id = kApiStopMove;
    req.parameter.clear();
    request_pub_->publish(req);
    RCLCPP_INFO(this->get_logger(), "cmd_vel timeout: sent StopMove (api_id=%d)",
                kApiStopMove);
  }

  void PublishMove(double vx, double vy, double wz) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = kApiMove;

    char param_buf[128];
    std::snprintf(param_buf, sizeof(param_buf), "{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f}",
                  vx, vy, wz);
    req.parameter = param_buf;

    request_pub_->publish(req);
  }

  static double Clamp(double value, double min_val, double max_val) {
    return std::min(std::max(value, min_val), max_val);
  }

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist::SharedPtr latest_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool is_stopped_;

  std::string cmd_vel_topic_;
  std::string request_topic_;
  double control_rate_hz_{};
  double cmd_timeout_sec_{};
  double max_linear_x_{};
  double max_linear_y_{};
  double max_angular_z_{};
  bool auto_stand_up_{};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2wCmdVelControlNode>());
  rclcpp::shutdown();
  return 0;
}
