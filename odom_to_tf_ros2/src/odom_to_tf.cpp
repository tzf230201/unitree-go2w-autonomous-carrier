#include <memory>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#define SCALE_X 1.33333
#define SCALE_Y 1.0
#define DEC_POINTS 1000

using std::placeholders::_1;

class OdomToTF : public rclcpp::Node {
    public:
        OdomToTF() : Node("odom_to_tf") {
            std::string odom_topic_1;
            std::string odom_topic_2;
            std::string pose_topic;
            std::string sport_mode_topic;
            odom_yaw_offset_ = this->declare_parameter("odom_yaw_offset", 0.0);
            frame_id = this->declare_parameter("frame_id", std::string("odom"));
            child_frame_id = this->declare_parameter("child_frame_id", std::string("base_footprint"));
            odom_topic_1 = this->declare_parameter("odom_topic_1", std::string("/lio_sam/mapping/odometry"));
            odom_topic_2 = this->declare_parameter("odom_topic_2", std::string(""));
            pose_topic = this->declare_parameter("pose_topic", std::string("/utlidar/robot_pose"));
            sport_mode_topic = this->declare_parameter("sport_mode_topic", std::string("/lf/sportmodestate"));
            RCLCPP_INFO(this->get_logger(), "odom_topic set to %s and %s",
                        odom_topic_1.c_str(), odom_topic_2.c_str());
            if (frame_id != "") {
                RCLCPP_INFO(this->get_logger(), "frame_id set to %s", frame_id.c_str());
            }
            else {
                RCLCPP_INFO(this->get_logger(), "frame_id was not set. The frame_id of the odom message will be used.");
            }
            if (child_frame_id != "") {
                RCLCPP_INFO(this->get_logger(), "child_frame_id set to %s", child_frame_id.c_str());
            }
            else {
                RCLCPP_INFO(this->get_logger(), "child_frame_id was not set. The child_frame_id of the odom message will be used.");
            }
            auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort();
            sub_1_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic_1, qos, std::bind(&OdomToTF::odomCallback_1, this, _1));
            if (!odom_topic_2.empty() && odom_topic_2 != odom_topic_1) {
                sub_2_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    odom_topic_2, qos, std::bind(&OdomToTF::odomCallback_2, this, _1));
            }
            if (!pose_topic.empty()) {
                sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    pose_topic, qos, std::bind(&OdomToTF::poseCallback, this, _1));
            }
            if (!sport_mode_topic.empty()) {
                sub_sport_mode_ = this->create_subscription<unitree_go::msg::SportModeState>(
                    sport_mode_topic, qos, std::bind(&OdomToTF::sportModeCallback, this, _1));
            }
            tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&OdomToTF::publishIdentityIfNoOdom, this));
        }
    private:
        std::string frame_id, child_frame_id;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_1_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_2_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
        rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sub_sport_mode_;
        geometry_msgs::msg::TransformStamped tfs_;
        nav_msgs::msg::Odometry lastOdom, relativeOdom;
        int freqDiv_1 = 0, freqDiv_2 = 0;
        bool initial_1 = true;
        bool initial_2 = true;
        bool have_odom_ = false;
        double odom_yaw_offset_ = 0.0;
        double initial_corrected_x_ = 0.0;
        double initial_corrected_y_ = 0.0;
        double initial_corrected_yaw_ = 0.0;
        bool initial_pose_ = true;
        bool initial_sport_mode_ = true;
        geometry_msgs::msg::Pose pose_origin_;
        float sport_origin_x_ = 0.0f;
        float sport_origin_y_ = 0.0f;
        float sport_origin_z_ = 0.0f;

        void publishIdentityIfNoOdom() {
            if (have_odom_) {
                return;
            }
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = this->now();
            tf.header.frame_id = frame_id;
            tf.child_frame_id = child_frame_id;
            tf.transform.translation.x = 0.0;
            tf.transform.translation.y = 0.0;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation.x = 0.0;
            tf.transform.rotation.y = 0.0;
            tf.transform.rotation.z = 0.0;
            tf.transform.rotation.w = 1.0;
            tfb_->sendTransform(tf);
        }

        void odomCallback_1(const nav_msgs::msg::Odometry::SharedPtr msg) {
            have_odom_ = true;
            tfs_.header = msg->header;
            // Use the source message timestamp so TF exists at the same time as sensor data.
            tfs_.header.stamp = msg->header.stamp;
            tfs_.header.frame_id = frame_id != "" ? frame_id : tfs_.header.frame_id;
            tfs_.child_frame_id = child_frame_id != "" ? child_frame_id : msg->child_frame_id;
            const double c = std::cos(odom_yaw_offset_);
            const double s = std::sin(odom_yaw_offset_);
            const double corrected_x = c * msg->pose.pose.position.x - s * msg->pose.pose.position.y;
            const double corrected_y = s * msg->pose.pose.position.x + c * msg->pose.pose.position.y;


            tf2::Quaternion current_orientation(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
            );
            tf2::Matrix3x3 current_matrix(current_orientation);

            double roll, pitch, current_yaw;
            current_matrix.getRPY(roll, pitch, current_yaw);

            const double corrected_yaw = current_yaw + odom_yaw_offset_;
            if (initial_1) {
                initial_corrected_x_ = corrected_x;
                initial_corrected_y_ = corrected_y;
                initial_corrected_yaw_ = corrected_yaw;
                initial_1 = false;
            }

            tfs_.transform.translation.x = corrected_x - initial_corrected_x_;
            tfs_.transform.translation.y = corrected_y - initial_corrected_y_;
            tfs_.transform.translation.z = msg->pose.pose.position.z;

            tf2::Quaternion q_corrected;
            q_corrected.setRPY(0.0, 0.0, corrected_yaw - initial_corrected_yaw_);
            tfs_.transform.rotation.x = q_corrected.x();
            tfs_.transform.rotation.y = q_corrected.y();
            tfs_.transform.rotation.z = q_corrected.z();
            tfs_.transform.rotation.w = q_corrected.w();

            tfb_->sendTransform(tfs_);
        }

        void odomCallback_2(const nav_msgs::msg::Odometry::SharedPtr msg) {
            have_odom_ = true;
            // auto &last_pose = lastOdom.pose.pose;
            // auto last_orientation = lastOdom.pose.pose.orientation;
            nav_msgs::msg::Odometry rel_pose;
            // auto cur_pose = msg->pose.pose;

            ++freqDiv_2;
            if (freqDiv_2 == 9)
                freqDiv_2 = 0;
            else
                return;

            if (!initial_2)
            {
                rel_pose.pose.pose.position.x = msg->pose.pose.position.x - lastOdom.pose.pose.position.x;
                rel_pose.pose.pose.position.y = msg->pose.pose.position.y - lastOdom.pose.pose.position.y;
                rel_pose.pose.pose.position.z = msg->pose.pose.position.z - lastOdom.pose.pose.position.z;

                tfs_.transform.translation.x += rel_pose.pose.pose.position.x;
                tfs_.transform.translation.y += rel_pose.pose.pose.position.y;
                // tfs_.transform.translation.z -= rel_pose.pose.pose.position.z;

                relativeOdom.pose.pose.position.x += ((float) ((int) (rel_pose.pose.pose.position.x * DEC_POINTS)) * SCALE_X) / DEC_POINTS;
                relativeOdom.pose.pose.position.y += ((float) ((int) (rel_pose.pose.pose.position.y * DEC_POINTS)) * SCALE_Y) / DEC_POINTS;

                std::cout << relativeOdom.pose.pose.position.x << " " << relativeOdom.pose.pose.position.y << " "  << std::endl;

                tf2::Quaternion current_orientation(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
                );

                tf2::Quaternion last_orientation(
                lastOdom.pose.pose.orientation.x,
                lastOdom.pose.pose.orientation.y,
                lastOdom.pose.pose.orientation.z,
                lastOdom.pose.pose.orientation.w
                );

                tf2::Quaternion real_orientation(
                tfs_.transform.rotation.x,
                tfs_.transform.rotation.y,
                tfs_.transform.rotation.z,
                tfs_.transform.rotation.w
                );

                tf2::Matrix3x3 current_matrix(current_orientation);
                tf2::Matrix3x3 last_matrix(last_orientation);
                tf2::Matrix3x3 real_matrix(real_orientation);

                double roll, pitch, current_yaw, last_yaw, real_yaw;
                current_matrix.getRPY(roll, pitch, current_yaw);
                last_matrix.getRPY(roll, pitch, last_yaw);
                real_matrix.getRPY(roll, pitch, real_yaw);

                // Compute relative yaw
                double relative_yaw = current_yaw - last_yaw;

                // Convert relative yaw back to a quaternion
                tf2::Quaternion relative_orientation;
                relative_orientation.setRPY(roll, pitch, real_yaw + relative_yaw);

                tfs_.transform.rotation.x = relative_orientation.x();
                tfs_.transform.rotation.y = relative_orientation.y();
                tfs_.transform.rotation.z = relative_orientation.z();
                tfs_.transform.rotation.w = relative_orientation.w();

                tfs_.header.stamp = this->now();
                tfb_->sendTransform(tfs_);
                // std::cout << current_yaw << " "
                //           << last_yaw << " "
                //           << real_yaw
                //           << std::endl;
                             

            }
            else 
            {
                relativeOdom.pose.pose.position.x = 0;
                relativeOdom.pose.pose.position.y = 0;
                relativeOdom.pose.pose.position.z = 0;
                relativeOdom.pose.pose.orientation.x = 0;
                relativeOdom.pose.pose.orientation.y = 0;
                relativeOdom.pose.pose.orientation.z = 0;
                relativeOdom.pose.pose.orientation.w = 1.0;
            }
            

            lastOdom.pose = msg->pose;
            initial_2 = false;
            // last_pose.position.x = msg->pose.pose.position.x;
            // last_pose.position.y = msg->pose.pose.position.y;
            // last_pose.position.z = msg->pose.pose.position.z;

            // last_pose.orientation.x = msg->pose.pose.orientation.x;
            // last_pose.orientation.y = msg->pose.pose.orientation.y;
            // last_pose.orientation.z = msg->pose.pose.orientation.z;
            // last_pose.orientation.w = msg->pose.pose.orientation.w;
        }

        void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            have_odom_ = true;
            if (initial_pose_) {
                pose_origin_ = msg->pose;
                initial_pose_ = false;
            }
            tfs_.header.stamp = msg->header.stamp;
            tfs_.header.frame_id = frame_id;
            tfs_.child_frame_id = child_frame_id;
            tfs_.transform.translation.x = msg->pose.position.x - pose_origin_.position.x;
            tfs_.transform.translation.y = msg->pose.position.y - pose_origin_.position.y;
            tfs_.transform.translation.z = msg->pose.position.z - pose_origin_.position.z;
            tfs_.transform.rotation = msg->pose.orientation;
            tfb_->sendTransform(tfs_);
        }

        void sportModeCallback(const unitree_go::msg::SportModeState::SharedPtr msg) {
            have_odom_ = true;
            if (initial_sport_mode_) {
                sport_origin_x_ = msg->position[0];
                sport_origin_y_ = msg->position[1];
                sport_origin_z_ = msg->position[2];
                initial_sport_mode_ = false;
            }
            // SportModeState does not provide a header; timestamp with node time.
            tfs_.header.stamp = this->now();
            tfs_.header.frame_id = frame_id;
            tfs_.child_frame_id = child_frame_id;
            tfs_.transform.translation.x = msg->position[0] - sport_origin_x_;
            tfs_.transform.translation.y = msg->position[1] - sport_origin_y_;
            tfs_.transform.translation.z = 0.0;
            tf2::Quaternion q_yaw;
            q_yaw.setRPY(0.0, 0.0, msg->imu_state.rpy[2]);
            tfs_.transform.rotation.x = q_yaw.x();
            tfs_.transform.rotation.y = q_yaw.y();
            tfs_.transform.rotation.z = q_yaw.z();
            tfs_.transform.rotation.w = q_yaw.w();
            tfb_->sendTransform(tfs_);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToTF>());
    rclcpp::shutdown();
    return 0;
}
