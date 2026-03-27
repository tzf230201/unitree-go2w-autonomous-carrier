#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
            frame_id = this->declare_parameter("frame_id", std::string("odom"));
            child_frame_id = this->declare_parameter("child_frame_id", std::string("base_footprint"));
            odom_topic_2 = this->declare_parameter("odom_topic_1", std::string("/lio_sam/mapping/odometry"));
            odom_topic_1 = this->declare_parameter("odom_topic_2", std::string("/dog_odom"));
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
            sub_1_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_1, rclcpp::SensorDataQoS(), std::bind(&OdomToTF::odomCallback_1, this, _1));
            sub_2_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_2, rclcpp::SensorDataQoS(), std::bind(&OdomToTF::odomCallback_2, this, _1));
            tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }
    private:
        std::string frame_id, child_frame_id;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_1_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_2_;
        geometry_msgs::msg::TransformStamped tfs_;
        double initial_pos_x = 0, initial_pos_y = 0, initial_pos_z = 0, initial_yaw = 0;
        nav_msgs::msg::Odometry lastOdom, relativeOdom;
        int freqDiv_1 = 0, freqDiv_2 = 0;
        bool initial_1 = true, initial_2 = true;
        tf2::Quaternion initial_orientation;

        void odomCallback_1(const nav_msgs::msg::Odometry::SharedPtr msg) {
            if (initial_1)
            {
                initial_pos_x = msg->pose.pose.position.x;
                initial_pos_y = msg->pose.pose.position.y;
                initial_pos_z = msg->pose.pose.position.z;

                initial_orientation = tf2::Quaternion(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w
                );
                tf2::Matrix3x3 initial_matrix(initial_orientation);
                double roll, pitch;
                initial_matrix.getRPY(roll, pitch, initial_yaw);

                initial_1 = false;

            }

            auto relative_pos_x = msg->pose.pose.position.x - initial_pos_x;
            auto relative_pos_y = msg->pose.pose.position.y - initial_pos_y;
            // auto relative_pos_z = msg->pose.pose.position.z - initial_pos_z;

            // auto relative_pos_x = msg->pose.pose.position.x;
            // auto relative_pos_y = msg->pose.pose.position.y;

            // std::cout << relative_pos_x << " " << relative_pos_y << " "  << std::endl;

            
            tfs_.header = msg->header;
            tfs_.header.stamp = this->now();
            tfs_.header.frame_id = frame_id != "" ? frame_id : tfs_.header.frame_id;
            tfs_.child_frame_id = child_frame_id != "" ? child_frame_id : msg->child_frame_id;
            tfs_.transform.translation.x = relative_pos_x;
            tfs_.transform.translation.y = relative_pos_y;
            tfs_.transform.translation.z = msg->pose.pose.position.z;

            // Convert input quaternions to tf2::Quaternion for manipulation
            tf2::Quaternion robot_quat(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
            );

            // tf2::fromMsg(msg->pose.pose.orientation, robot_quat);
            // tf2::fromMsg(reference_orientation, initial_orientation);

            // Compute the inverse of the reference orientation
            tf2::Quaternion reference_quat_inv = initial_orientation.inverse();

            // Compute the relative orientation
            tf2::Quaternion relative_quat = reference_quat_inv * robot_quat;

            // Normalize the result to avoid numerical drift
            relative_quat.normalize();

            // Convert the result back to geometry_msgs::msg::Quaternion
            // if (++freqDiv_1 == 1)
            // {
            //     tfs_.transform.rotation.x = relative_quat.x();
            //     tfs_.transform.rotation.y = relative_quat.y();
            //     tfs_.transform.rotation.z = relative_quat.z();
            //     tfs_.transform.rotation.w = relative_quat.w();
            //     freqDiv_1 = 0;
            // }
            


            tf2::Quaternion current_orientation(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
            );
            tf2::Matrix3x3 current_matrix(current_orientation);

            double roll, pitch, current_yaw;
            current_matrix.getRPY(roll, pitch, current_yaw);

            // Compute relative yaw
            double relative_yaw = current_yaw - initial_yaw;

            // Convert relative yaw back to a quaternion
            tf2::Quaternion relative_orientation;
            relative_orientation.setRPY(roll, pitch, relative_yaw);

            if (++freqDiv_1 == 1)
            {
                freqDiv_1 = 0;
                tfs_.transform.rotation = msg->pose.pose.orientation;
            }

            // Set the relative orientation to the transform
            // tfs_.transform.rotation.x = relative_orientation.x();
            // tfs_.transform.rotation.y = relative_orientation.y();
            // tfs_.transform.rotation.z = relative_orientation.z();
            // tfs_.transform.rotation.w = relative_orientation.w();

            tfb_->sendTransform(tfs_);
            // tfs_.header.frame_id = "base_footprint";
            // tfs_.child_frame_id = "odom1";
            // tfb_->sendTransform(tfs_);
        }

        void odomCallback_2(const nav_msgs::msg::Odometry::SharedPtr msg) {
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
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToTF>());
    rclcpp::shutdown();
    return 0;
}
