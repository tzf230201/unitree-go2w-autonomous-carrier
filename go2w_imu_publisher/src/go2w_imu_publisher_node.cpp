#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "unitree_go/msg/low_state.hpp"

class Go2wImuPublisher : public rclcpp::Node {
public:
  Go2wImuPublisher() : Node("go2w_imu_publisher") {
    input_topic_ = this->declare_parameter<std::string>("input_lowstate_topic", "/lowstate");
    output_topic_ = this->declare_parameter<std::string>("output_imu_topic", "/dog_imu_raw");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "imu_link");

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(output_topic_, 100);

    lowstate_sub_ = this->create_subscription<unitree_go::msg::LowState>(
        input_topic_, 100,
        std::bind(&Go2wImuPublisher::lowStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Publishing IMU from '%s' to '%s' (frame_id='%s')",
                input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str());
  }

private:
  void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg) {
    sensor_msgs::msg::Imu imu_msg;

    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = frame_id_;

    // Unitree order in LowState IMU quaternion is [w, x, y, z].
    imu_msg.orientation.w = msg->imu_state.quaternion[0];
    imu_msg.orientation.x = msg->imu_state.quaternion[1];
    imu_msg.orientation.y = msg->imu_state.quaternion[2];
    imu_msg.orientation.z = msg->imu_state.quaternion[3];

    imu_msg.angular_velocity.x = msg->imu_state.gyroscope[0];
    imu_msg.angular_velocity.y = msg->imu_state.gyroscope[1];
    imu_msg.angular_velocity.z = msg->imu_state.gyroscope[2];

    imu_msg.linear_acceleration.x = msg->imu_state.accelerometer[0];
    imu_msg.linear_acceleration.y = msg->imu_state.accelerometer[1];
    imu_msg.linear_acceleration.z = msg->imu_state.accelerometer[2];

    imu_pub_->publish(imu_msg);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;

  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2wImuPublisher>());
  rclcpp::shutdown();
  return 0;
}
