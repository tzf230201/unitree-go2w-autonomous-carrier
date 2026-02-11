#include "Go2JointStatePublisher.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Go2JointStatePublisher>("g2w_joints_state_publisher");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0; 
}