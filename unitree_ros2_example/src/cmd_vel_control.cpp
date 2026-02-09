#include <iostream>
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
        // コマンド速度トピックを購読
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&CmdVelToSportRequest::cmdVelCallback, this, _1));

        // リクエスト送信用パブリッシャ
        request_publisher_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg)
    {
        // 受け取った/cmd_velメッセージを出力
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel: LinearX=%f, LinearY=%f, AngularZ=%f",
                    cmd_vel_msg->linear.x, cmd_vel_msg->linear.y, cmd_vel_msg->angular.z);

        // /cmd_velメッセージをスポーツリクエストに変換
        unitree_api::msg::Request req;
        sport_client_.Move(req, cmd_vel_msg->linear.x, cmd_vel_msg->linear.y, cmd_vel_msg->angular.z);

        // リクエストを送信
        request_publisher_->publish(req);
        // RCLCPP_INFO(this->get_logger(), "Published Request: LinearX=%f, LinearY=%f, AngularZ=%f",
        //             cmd_vel_msg->linear.x, cmd_vel_msg->linear.y, cmd_vel_msg->angular.z);
    }

    // 購読者
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;

    // パブリッシャ
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_publisher_;

    // SportClientインスタンス
    SportClient sport_client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelToSportRequest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
