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
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "input_cloud", rclcpp::SensorDataQoS(),
            std::bind(&HesaiToLioSamConverter::callback, this, std::placeholders::_1));

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "output_cloud", rclcpp::SensorDataQoS());

        RCLCPP_INFO(get_logger(), "HesaiToLioSamConverter started");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        const size_t n = static_cast<size_t>(msg->width) * msg->height;
        if (n == 0) return;

        // --- Pass 1: find scan start time (minimum per-point timestamp) ---
        double t0 = std::numeric_limits<double>::max();
        {
            sensor_msgs::PointCloud2ConstIterator<double> it(*msg, "timestamp");
            for (size_t i = 0; i < n; ++i, ++it) {
                if (*it < t0) t0 = *it;
            }
        }

        // --- Build output cloud: same fields but 'time' (float32) instead of 'timestamp' (float64) ---
        sensor_msgs::msg::PointCloud2 out;
        out.header      = msg->header;
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
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HesaiToLioSamConverter>());
    rclcpp::shutdown();
    return 0;
}
