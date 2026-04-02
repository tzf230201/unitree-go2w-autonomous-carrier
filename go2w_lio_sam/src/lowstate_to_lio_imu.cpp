#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <unitree_go/msg/low_state.hpp>
#include <cmath>

class LowStateToLioImu : public rclcpp::Node
{
public:
    LowStateToLioImu() : Node("lowstate_to_lio_imu")
    {
        input_topic_ = declare_parameter<std::string>("input_lowstate_topic", "/lowstate");
        output_topic_ = declare_parameter<std::string>("output_imu_topic", "/lio_imu_raw");
        frame_id_ = declare_parameter<std::string>("frame_id", "imu");
        use_rpy_orientation_ = declare_parameter<bool>("use_rpy_orientation", false);
        calibration_samples_ = declare_parameter<int>("calibration_samples", 500);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(500)).reliable();

        pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, qos);
        sub_ = create_subscription<unitree_go::msg::LowState>(
            input_topic_,
            rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
            std::bind(&LowStateToLioImu::callback, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "LowStateToLioImu started: %s -> %s",
            input_topic_.c_str(),
            output_topic_.c_str());
    }

private:
    static double meanFromAccumulator(double sum, size_t count)
    {
        return count == 0 ? 0.0 : sum / static_cast<double>(count);
    }

    static void accelToRollPitch(double ax, double ay, double az, double & roll, double & pitch)
    {
        roll = std::atan2(ay, az);
        pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    }

    void finishCalibration()
    {
        if (calibration_done_ || calibration_count_ == 0) {
            calibration_done_ = true;
            return;
        }

        gyro_bias_x_ = meanFromAccumulator(sum_gx_, calibration_count_);
        gyro_bias_y_ = meanFromAccumulator(sum_gy_, calibration_count_);
        gyro_bias_z_ = meanFromAccumulator(sum_gz_, calibration_count_);

        const double mean_ax = meanFromAccumulator(sum_ax_, calibration_count_);
        const double mean_ay = meanFromAccumulator(sum_ay_, calibration_count_);
        const double mean_az = meanFromAccumulator(sum_az_, calibration_count_);
        accel_bias_x_ = mean_ax;
        accel_bias_y_ = mean_ay;
        const double mean_roll = meanFromAccumulator(sum_roll_, calibration_count_);
        const double mean_pitch = meanFromAccumulator(sum_pitch_, calibration_count_);

        double accel_roll = 0.0;
        double accel_pitch = 0.0;
        accelToRollPitch(mean_ax, mean_ay, mean_az, accel_roll, accel_pitch);

        const double d_roll = accel_roll - mean_roll;
        const double d_pitch = accel_pitch - mean_pitch;
        orientation_correction_.setRPY(d_roll, d_pitch, 0.0);
        orientation_correction_.normalize();

        calibration_done_ = true;

        RCLCPP_INFO(
            get_logger(),
            "IMU calibration complete with %zu samples: gyro_bias=(%.6f, %.6f, %.6f), "
            "accel_bias_xy=(%.6f, %.6f), orientation_rp_correction=(%.6f, %.6f)",
            calibration_count_,
            gyro_bias_x_,
            gyro_bias_y_,
            gyro_bias_z_,
            accel_bias_x_,
            accel_bias_y_,
            d_roll,
            d_pitch);
    }

    void callback(const unitree_go::msg::LowState::SharedPtr msg)
    {
        sensor_msgs::msg::Imu out;
        auto stamp = now();
        auto stamp_ns = stamp.nanoseconds();
        if (tick_time_initialized_ && msg->tick <= last_tick_) {
            ++dropped_stale_tick_count_;
            if (dropped_stale_tick_count_ <= 5 || dropped_stale_tick_count_ % 100 == 0) {
                RCLCPP_WARN(
                    get_logger(),
                    "Dropping stale LowState tick (%u -> %u, count=%zu)",
                    last_tick_,
                    msg->tick,
                    dropped_stale_tick_count_);
                }
            return;
        }
        tick_time_initialized_ = true;
        last_tick_ = msg->tick;

        if (last_stamp_ns_ != 0 && stamp_ns <= last_stamp_ns_) {
            ++non_monotonic_stamp_count_;
            stamp_ns = last_stamp_ns_ + 1;
            if (non_monotonic_stamp_count_ <= 5 || non_monotonic_stamp_count_ % 100 == 0) {
                RCLCPP_WARN(
                    get_logger(),
                    "Adjusted non-monotonic IMU stamp from ROS clock (count=%zu)",
                    non_monotonic_stamp_count_);
            }
        }
        last_stamp_ns_ = stamp_ns;
        out.header.stamp = rclcpp::Time(stamp_ns, stamp.get_clock_type());
        out.header.frame_id = frame_id_;

        tf2::Quaternion q;
        if (use_rpy_orientation_) {
            q.setRPY(
                msg->imu_state.rpy[0],
                msg->imu_state.rpy[1],
                msg->imu_state.rpy[2]);
        } else {
            q = tf2::Quaternion(
                msg->imu_state.quaternion[1],
                msg->imu_state.quaternion[2],
                msg->imu_state.quaternion[3],
                msg->imu_state.quaternion[0]);
        }
        q.normalize();

        const double raw_ax = msg->imu_state.accelerometer[0];
        const double raw_ay = msg->imu_state.accelerometer[1];
        const double raw_az = msg->imu_state.accelerometer[2];
        const double raw_gx = msg->imu_state.gyroscope[0];
        const double raw_gy = msg->imu_state.gyroscope[1];
        const double raw_gz = msg->imu_state.gyroscope[2];

        if (!calibration_done_) {
            ++calibration_count_;
            sum_ax_ += raw_ax;
            sum_ay_ += raw_ay;
            sum_az_ += raw_az;
            sum_gx_ += raw_gx;
            sum_gy_ += raw_gy;
            sum_gz_ += raw_gz;

            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            sum_roll_ += roll;
            sum_pitch_ += pitch;

            if (calibration_count_ < static_cast<size_t>(calibration_samples_)) {
                return;
            }
            finishCalibration();
        }

        q *= orientation_correction_;
        q.normalize();
        out.orientation.x = q.x();
        out.orientation.y = q.y();
        out.orientation.z = q.z();
        out.orientation.w = q.w();

        out.angular_velocity.x = raw_gx - gyro_bias_x_;
        out.angular_velocity.y = raw_gy - gyro_bias_y_;
        out.angular_velocity.z = raw_gz - gyro_bias_z_;

        out.linear_acceleration.x = raw_ax - accel_bias_x_;
        out.linear_acceleration.y = raw_ay - accel_bias_y_;
        // Z is not bias-corrected: mean_az ≈ gravity (9.8 m/s²), which LIO-SAM
        // needs intact for gravity-vector initialization. Only X/Y sensor offsets
        // (≈0 on flat ground) are removed.
        out.linear_acceleration.z = raw_az;

        pub_->publish(out);
    }

    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    std::string input_topic_;
    std::string output_topic_;
    std::string frame_id_;
    bool use_rpy_orientation_;
    int calibration_samples_;
    bool tick_time_initialized_ = false;
    uint32_t last_tick_ = 0;
    int64_t last_stamp_ns_ = 0;
    size_t dropped_stale_tick_count_ = 0;
    size_t non_monotonic_stamp_count_ = 0;
    bool calibration_done_ = false;
    size_t calibration_count_ = 0;
    double sum_ax_ = 0.0;
    double sum_ay_ = 0.0;
    double sum_az_ = 0.0;
    double sum_gx_ = 0.0;
    double sum_gy_ = 0.0;
    double sum_gz_ = 0.0;
    double sum_roll_ = 0.0;
    double sum_pitch_ = 0.0;
    double gyro_bias_x_ = 0.0;
    double gyro_bias_y_ = 0.0;
    double gyro_bias_z_ = 0.0;
    double accel_bias_x_ = 0.0;
    double accel_bias_y_ = 0.0;
    tf2::Quaternion orientation_correction_{0.0, 0.0, 0.0, 1.0};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LowStateToLioImu>());
    rclcpp::shutdown();
    return 0;
}
