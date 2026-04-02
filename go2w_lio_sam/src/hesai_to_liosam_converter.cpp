/**
 * Converts Hesai PointCloud2 format to LIO-SAM Velodyne-compatible format.
 *
 * Hesai driver publishes:
 *   ring      → UINT16
 *   timestamp → FLOAT64  (absolute time in seconds, per-point)
 *
 * LIO-SAM (sensor: velodyne) expects:
 *   ring → UINT16
 *   time → FLOAT32  (relative time from scan start, in seconds)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <limits>

class HesaiToLioSamConverter : public rclcpp::Node
{
public:
    HesaiToLioSamConverter() : Node("hesai_to_liosam_converter")
    {
        auto input_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        auto output_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
        output_frame_ = declare_parameter<std::string>("output_frame", "lidar_link");

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "input_cloud", input_qos,
            std::bind(&HesaiToLioSamConverter::callback, this, std::placeholders::_1));

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "output_cloud", output_qos);

        RCLCPP_INFO(get_logger(), "HesaiToLioSamConverter started");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        const size_t n = static_cast<size_t>(msg->width) * msg->height;
        if (n == 0) return;

        // --- Pass 1: find scan start (t0) and scan end (t_max) hardware timestamps ---
        double t0   = std::numeric_limits<double>::max();
        double t_max = std::numeric_limits<double>::lowest();
        {
            sensor_msgs::PointCloud2ConstIterator<double> it(*msg, "timestamp");
            for (size_t i = 0; i < n; ++i, ++it) {
                if (*it < t0)    t0    = *it;
                if (*it > t_max) t_max = *it;
            }
        }

        // --- Build output cloud: same fields but 'time' (float32) instead of 'timestamp' (float64) ---
        sensor_msgs::msg::PointCloud2 out;
        out.header = msg->header;

        // The Hesai hardware clock runs behind the system (ROS) clock by a fixed
        // offset (~1-2 s on this platform).  LIO-SAM's imageProjection matches the
        // cloud header stamp against IMU timestamps, which are on the system clock.
        // Re-stamp the header so that its scan-start time is expressed in system
        // time: place it (scan_duration) seconds before now(), so that
        //   timeScanCur  = now() - scan_duration   (≈ system time of first point)
        //   timeScanEnd  = timeScanCur + scan_duration ≈ now()
        // and the IMU queue (also on the system clock) fully brackets the scan.
        const double scan_duration = t_max - t0;
        out.header.stamp = this->now() - rclcpp::Duration::from_seconds(scan_duration);
        out.header.frame_id = output_frame_;
        out.height      = 1;
        out.width       = static_cast<uint32_t>(n);
        out.is_dense    = msg->is_dense;
        out.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier mod(out);
        mod.setPointCloud2Fields(6,
            "x",         1, sensor_msgs::msg::PointField::FLOAT32,
            "y",         1, sensor_msgs::msg::PointField::FLOAT32,
            "z",         1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
            "ring",      1, sensor_msgs::msg::PointField::UINT16,
            "time",      1, sensor_msgs::msg::PointField::FLOAT32);
        mod.resize(n);

        // --- Pass 2: copy and convert ---
        sensor_msgs::PointCloud2ConstIterator<float>    in_x(*msg,   "x");
        sensor_msgs::PointCloud2ConstIterator<float>    in_y(*msg,   "y");
        sensor_msgs::PointCloud2ConstIterator<float>    in_z(*msg,   "z");
        sensor_msgs::PointCloud2ConstIterator<float>    in_i(*msg,   "intensity");
        sensor_msgs::PointCloud2ConstIterator<uint16_t> in_r(*msg,   "ring");
        sensor_msgs::PointCloud2ConstIterator<double>   in_t(*msg,   "timestamp");

        sensor_msgs::PointCloud2Iterator<float>    out_x(out,  "x");
        sensor_msgs::PointCloud2Iterator<float>    out_y(out,  "y");
        sensor_msgs::PointCloud2Iterator<float>    out_z(out,  "z");
        sensor_msgs::PointCloud2Iterator<float>    out_i(out,  "intensity");
        sensor_msgs::PointCloud2Iterator<uint16_t> out_r(out,  "ring");
        sensor_msgs::PointCloud2Iterator<float>    out_t(out,  "time");

        for (size_t i = 0; i < n;
             ++i,
             ++in_x, ++in_y, ++in_z, ++in_i, ++in_r, ++in_t,
             ++out_x, ++out_y, ++out_z, ++out_i, ++out_r, ++out_t)
        {
            *out_x = *in_x;
            *out_y = *in_y;
            *out_z = *in_z;
            *out_i = *in_i;
            *out_r = *in_r;
            *out_t = static_cast<float>(*in_t - t0);
        }

        pub_->publish(out);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    std::string output_frame_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HesaiToLioSamConverter>());
    rclcpp::shutdown();
    return 0;
}
