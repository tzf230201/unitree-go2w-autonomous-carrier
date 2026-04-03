// Converts Hesai PointCloud2 (x,y,z,intensity,ring,timestamp[float64])
// to Velodyne-style PointCloud2 (x,y,z,intensity,ring,time[float32])
// Single pass: uses header stamp as t0, computes offset per point.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class HesaiToVelodyneConverter : public rclcpp::Node
{
public:
  HesaiToVelodyneConverter() : Node("hesai_to_velodyne_converter")
  {
    auto input_topic = declare_parameter<std::string>("input_topic", "/lidar_points");
    auto output_topic = declare_parameter<std::string>("output_topic", "/velodyne_points");

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, 10,
        std::bind(&HesaiToVelodyneConverter::callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Converting %s -> %s", input_topic.c_str(), output_topic.c_str());
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const uint32_t point_step = msg->point_step;
    const uint32_t n_points = msg->width * msg->height;
    if (n_points == 0) return;

    // Resolve input field offsets once
    if (!offsets_resolved_) {
      for (const auto &f : msg->fields) {
        if (f.name == "x") in_x_ = f.offset;
        else if (f.name == "y") in_y_ = f.offset;
        else if (f.name == "z") in_z_ = f.offset;
        else if (f.name == "intensity") in_i_ = f.offset;
        else if (f.name == "ring") in_ring_ = f.offset;
        else if (f.name == "timestamp") in_ts_ = f.offset;
      }
      if (in_ts_ < 0) {
        RCLCPP_WARN(get_logger(), "No 'timestamp' field — passing through");
        passthrough_ = true;
      }
      offsets_resolved_ = true;
    }

    if (passthrough_) {
      pub_->publish(*msg);
      return;
    }

    // Use header stamp as t0 (frame start time set by Hesai driver)
    const double t0 = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    // Output layout: x(4) y(4) z(4) intensity(4) ring(2) time(4) = 22 bytes
    constexpr uint32_t OUT_STEP = 22;

    // Reuse output message buffer to avoid reallocation
    out_.header = msg->header;
    out_.height = 1;
    out_.width = n_points;
    out_.is_bigendian = false;
    out_.is_dense = msg->is_dense;
    out_.point_step = OUT_STEP;
    out_.row_step = n_points * OUT_STEP;

    if (out_.fields.empty()) {
      out_.fields.resize(6);
      int off = 0;
      auto set = [&](int idx, const char *name, uint8_t dt, uint32_t sz) {
        out_.fields[idx].name = name;
        out_.fields[idx].offset = off;
        out_.fields[idx].datatype = dt;
        out_.fields[idx].count = 1;
        off += sz;
      };
      set(0, "x",         sensor_msgs::msg::PointField::FLOAT32, 4);
      set(1, "y",         sensor_msgs::msg::PointField::FLOAT32, 4);
      set(2, "z",         sensor_msgs::msg::PointField::FLOAT32, 4);
      set(3, "intensity", sensor_msgs::msg::PointField::FLOAT32, 4);
      set(4, "ring",      sensor_msgs::msg::PointField::UINT16,  2);
      set(5, "time",      sensor_msgs::msg::PointField::FLOAT32, 4);
    }

    const uint32_t total = n_points * OUT_STEP;
    out_.data.resize(total);

    // Single pass: bulk copy first 18 bytes (x,y,z,intensity,ring are
    // identical layout), only convert timestamp (float64 -> float32 offset)
    const uint8_t *src = msg->data.data();
    uint8_t *dst = out_.data.data();

    // Check if Hesai layout is contiguous: x(0) y(4) z(8) i(12) ring(16) ts(18)
    // If so, we can bulk copy the first 18 bytes per point
    const bool contiguous = (in_x_ == 0 && in_y_ == 4 && in_z_ == 8 &&
                             in_i_ == 12 && in_ring_ == 16 && in_ts_ == 18);

    if (contiguous) {
      for (uint32_t i = 0; i < n_points; ++i) {
        const uint8_t *in = src + i * point_step;
        uint8_t *o = dst + i * OUT_STEP;

        std::memcpy(o, in, 18);               // x,y,z,intensity,ring in one shot

        double t_abs;
        std::memcpy(&t_abs, in + 18, 8);
        float t_offset = static_cast<float>(t_abs - t0);
        std::memcpy(o + 18, &t_offset, 4);   // time
      }
    } else {
      for (uint32_t i = 0; i < n_points; ++i) {
        const uint8_t *in = src + i * point_step;
        uint8_t *o = dst + i * OUT_STEP;

        std::memcpy(o,      in + in_x_,    4);
        std::memcpy(o + 4,  in + in_y_,    4);
        std::memcpy(o + 8,  in + in_z_,    4);
        std::memcpy(o + 12, in + in_i_,    4);
        std::memcpy(o + 16, in + in_ring_, 2);

        double t_abs;
        std::memcpy(&t_abs, in + in_ts_, 8);
        float t_offset = static_cast<float>(t_abs - t0);
        std::memcpy(o + 18, &t_offset, 4);
      }
    }

    pub_->publish(out_);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

  // Cached state
  sensor_msgs::msg::PointCloud2 out_;
  bool offsets_resolved_ = false;
  bool passthrough_ = false;
  int in_x_ = 0, in_y_ = 4, in_z_ = 8, in_i_ = 12, in_ring_ = 16, in_ts_ = -1;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HesaiToVelodyneConverter>());
  rclcpp::shutdown();
  return 0;
}
