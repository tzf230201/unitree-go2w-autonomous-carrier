#ifndef GO2JOINTSTATEPUBLISHER
#define GO2JOINTSTATEPUBLISHER

#define NUM_OF_JOINTS 16
#define FREQ_DIV 1

#include <stdio.h>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/joint_state.hpp>


#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/motor_state.hpp"
#include "unitree_go/msg/low_state.hpp"

#include "motor_crc.h"

class Go2JointStatePublisher : public rclcpp::Node
{
    public:
    Go2JointStatePublisher(std::string nodeName);

    void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);

    void publishJointsState();

private:
    // unitree_go::msg::LowCmd lowCmd;

    rclcpp::Clock::SharedPtr clock_ptr;

    rclcpp::Clock steady_clock;

    rclcpp::Clock clock;
    double sec;
    int64_t nanosec;
    int freqDivCount;

    // Index mapping for LowState motor_state
    // FR_0->0, FR_1->1, FR_2->2
    // FL_0->3, FL_1->4, FL_2->5
    // RR_0->6, RR_1->7, RR_2->8
    // RL_0->9, RL_1->10, RL_2->11
    // FR_Wheel->12, FL_Wheel->13, RR_Wheel->14, RL_Wheel->15
    std::string jointNames[NUM_OF_JOINTS] = {
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "FR_foot_joint",
    "FL_foot_joint",
    "RR_foot_joint",
    "RL_foot_joint"
    };

    double jointPositions[NUM_OF_JOINTS];

    unitree_go::msg::LowState lowState;

    sensor_msgs::msg::JointState jointState;

    //rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr lowState_pub;
    // rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr servo_pub;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowState_sub;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointsState_pub;
    // rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_subscriber_;
    std::string jointsStateTopic = "/joint_states";
    std::string lowStateTopic = "/lowstate";


};

 
#endif //GO2JOINTSTATEPUBLISHER