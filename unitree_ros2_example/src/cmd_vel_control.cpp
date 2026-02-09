#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "common/ros2_sport_client.h"

using namespace std::placeholders;

class CmdVelToSportRequest : public rclcpp::Node
{
public:
    CmdVelToSportRequest() : Node("cmd_vel_to_sport_request")
    {
        using namespace std::chrono_literals;

        cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        sport_request_topic_ = this->declare_parameter<std::string>("sport_request_topic", "/api/sport/request");
        max_linear_x_ = this->declare_parameter<double>("max_linear_x", 1.0);
        max_linear_y_ = this->declare_parameter<double>("max_linear_y", 0.6);
        max_angular_z_ = this->declare_parameter<double>("max_angular_z", 1.5);
        cmd_timeout_sec_ = this->declare_parameter<double>("cmd_timeout_sec", 0.5);

        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, rclcpp::QoS(10), std::bind(&CmdVelToSportRequest::cmdVelCallback, this, _1));

        request_publisher_ = this->create_publisher<unitree_api::msg::Request>(sport_request_topic_, rclcpp::QoS(10));

        last_cmd_time_ = this->now();
        watchdog_timer_ = this->create_wall_timer(100ms, std::bind(&CmdVelToSportRequest::watchdogTick, this));

        RCLCPP_INFO(this->get_logger(),
                    "cmd_vel_control started. cmd_vel_topic=%s sport_request_topic=%s timeout=%.3fs limits(x=%.2f y=%.2f wz=%.2f)",
                    cmd_vel_topic_.c_str(), sport_request_topic_.c_str(), cmd_timeout_sec_,
                    max_linear_x_, max_linear_y_, max_angular_z_);
    }

private:
    static double clampAbs(double value, double abs_limit)
    {
        if (abs_limit <= 0.0) {
            return 0.0;
        }
        if (value > abs_limit) {
            return abs_limit;
        }
        if (value < -abs_limit) {
            return -abs_limit;
        }
        return value;
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg)
    {
        last_cmd_time_ = this->now();

        last_linear_x_ = clampAbs(cmd_vel_msg->linear.x, max_linear_x_);
        last_linear_y_ = clampAbs(cmd_vel_msg->linear.y, max_linear_y_);
        last_angular_z_ = clampAbs(cmd_vel_msg->angular.z, max_angular_z_);

        RCLCPP_INFO(this->get_logger(), "Received cmd_vel (clamped): LinearX=%f, LinearY=%f, AngularZ=%f",
                    last_linear_x_, last_linear_y_, last_angular_z_);

        publishMove(last_linear_x_, last_linear_y_, last_angular_z_);
        // RCLCPP_INFO(this->get_logger(), "Published Request: LinearX=%f, LinearY=%f, AngularZ=%f",
        //             cmd_vel_msg->linear.x, cmd_vel_msg->linear.y, cmd_vel_msg->angular.z);
    }

    void watchdogTick()
    {
        if (cmd_timeout_sec_ <= 0.0) {
            return;
        }

        const auto age = (this->now() - last_cmd_time_).seconds();
        if (age <= cmd_timeout_sec_) {
            stale_stop_sent_ = false;
            return;
        }

        if (stale_stop_sent_) {
            return;
        }
        stale_stop_sent_ = true;
        RCLCPP_WARN(this->get_logger(), "cmd_vel timeout (age=%.3fs > %.3fs). Sending STOP.", age, cmd_timeout_sec_);
        publishMove(0.0, 0.0, 0.0);
    }

    void publishMove(double linear_x, double linear_y, double angular_z)
    {
        unitree_api::msg::Request req;
        sport_client_.Move(req, linear_x, linear_y, angular_z);
        request_publisher_->publish(req);
    }

    // 購読者
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;

    // パブリッシャ
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_publisher_;

    // SportClientインスタンス
    SportClient sport_client_;

    std::string cmd_vel_topic_;
    std::string sport_request_topic_;
    double max_linear_x_{1.0};
    double max_linear_y_{0.6};
    double max_angular_z_{1.5};
    double cmd_timeout_sec_{0.5};

    rclcpp::Time last_cmd_time_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    double last_linear_x_{0.0};
    double last_linear_y_{0.0};
    double last_angular_z_{0.0};
    bool stale_stop_sent_{false};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelToSportRequest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
