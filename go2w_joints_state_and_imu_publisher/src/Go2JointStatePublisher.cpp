#include "Go2JointStatePublisher.h"


Go2JointStatePublisher::Go2JointStatePublisher(std::string nodeName)
: Node(nodeName), // clock(RCL_ROS_TIME)
  steady_clock(RCL_STEADY_TIME)
{
    lowStateTopic = this->declare_parameter<std::string>("input_lowstate_topic", lowStateTopic);
    jointsStateTopic = this->declare_parameter<std::string>("output_joint_states_topic", jointsStateTopic);
    imuTopic = this->declare_parameter<std::string>("output_imu_topic", imuTopic);
    imuFrameId = this->declare_parameter<std::string>("frame_id", imuFrameId);

    lowState_sub = this->create_subscription<unitree_go::msg::LowState>(
        lowStateTopic, 10, std::bind(&Go2JointStatePublisher::lowStateCallback, this, std::placeholders::_1));

    jointsState_pub = this->create_publisher<sensor_msgs::msg::JointState>(jointsStateTopic, 500);
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imuTopic, 500);

    jointState.name.resize(NUM_OF_JOINTS);     // Names of the joints
    jointState.position.resize(NUM_OF_JOINTS); // Joint positions
    // jointState.velocity.resize(NUM_OF_JOINTS); // Joint velocities
    // jointState.effort.resize(NUM_OF_JOINTS);   // Joint efforts

    freqDivCount = 0;
    clock_ptr = this->get_clock();

    for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
    { 
        jointState.name[i] = jointNames[i];
    }


    /* for (int i = 0; i < 20; i++)
        {
            lowCmd.motor_cmd[i].mode = 0x01; //Set toque mode, 0x00 is passive mode
            lowCmd.motor_cmd[i].q = (2.146E+9f);
            lowCmd.motor_cmd[i].kp = 0;
            lowCmd.motor_cmd[i].dq = (16000.0f);
            lowCmd.motor_cmd[i].kd = 0;
            lowCmd.motor_cmd[i].tau = 0;
        }

    for(int i = 0; i < 4; i++)
    {
        lowCmd.motor_cmd[i*3+0].mode = 0x01;
        lowCmd.motor_cmd[i*3+0].kp = 100;
        lowCmd.motor_cmd[i*3+0].dq = 0;
        lowCmd.motor_cmd[i*3+0].kd = 5;
        lowCmd.motor_cmd[i*3+0].tau = 0;
        lowCmd.motor_cmd[i*3+1].mode = 0x01;
        lowCmd.motor_cmd[i*3+1].kp = 100;
        lowCmd.motor_cmd[i*3+1].dq = 0;
        lowCmd.motor_cmd[i*3+1].kd = 5;
        lowCmd.motor_cmd[i*3+1].tau = 0;
        lowCmd.motor_cmd[i*3+2].mode = 0x01;
        lowCmd.motor_cmd[i*3+2].kp = 100;
        lowCmd.motor_cmd[i*3+2].dq = 0;
        lowCmd.motor_cmd[i*3+2].kd = 5;
        lowCmd.motor_cmd[i*3+2].tau =0;
    } */
}

void Go2JointStatePublisher::publishJointsState()
{
    jointState.header.stamp.sec = sec;
    jointState.header.stamp.nanosec = nanosec;

    jointsState_pub->publish(jointState);
}

void Go2JointStatePublisher::publishImu()
{
    imuState.header.stamp.sec = sec;
    imuState.header.stamp.nanosec = nanosec;
    imuState.header.frame_id = imuFrameId;

    imu_pub->publish(imuState);
}

void Go2JointStatePublisher::lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg)
{
    ++freqDivCount;

    if (freqDivCount < FREQ_DIV)
        return;

    for (int i = 0; i < NUM_OF_JOINTS; i++)
        jointState.position[i] = msg->motor_state[i].q;

    imuState.orientation.w = msg->imu_state.quaternion[0];
    imuState.orientation.x = msg->imu_state.quaternion[1];
    imuState.orientation.y = msg->imu_state.quaternion[2];
    imuState.orientation.z = msg->imu_state.quaternion[3];

    imuState.angular_velocity.x = msg->imu_state.gyroscope[0];
    imuState.angular_velocity.y = msg->imu_state.gyroscope[1];
    imuState.angular_velocity.z = msg->imu_state.gyroscope[2];

    imuState.linear_acceleration.x = msg->imu_state.accelerometer[0];
    imuState.linear_acceleration.y = msg->imu_state.accelerometer[1];
    imuState.linear_acceleration.z = msg->imu_state.accelerometer[2];

    // rclcpp::Time now = clock.now();
    // rclcpp::Time now = steady_clock.now();
    rclcpp::Time now = clock_ptr->now();
    sec = now.seconds();
    nanosec = now.nanoseconds() % 1000000000; // get only the nano seconds part of the returned value
    publishJointsState();
    publishImu();
    freqDivCount = 0;
}
