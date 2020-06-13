
#include <ros/ros.h>
#include "walker_webots_hardware_interface.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "walker_webots_hardware_interface");
    // ros::CallbackQueue ros_queue;

    ros::NodeHandle nh;
    // nh.setCallbackQueue(&ros_queue);
    ROS_INFO("Starting hardware interface...");
    walker_webots_hardware_interface::WalkerWebotsHardwareInterface rhi(nh);

    // ros::MultiThreadedSpinner spinner(0);
    // spinner.spin(&ros_queue);

    ROS_INFO("Spinning...");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
