#include <stdlib.h>
#include <std_msgs/String.h>
#include <tough_controller_interface/chest_control_interface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "chest_control");
    ros::NodeHandle nh;
    ROS_INFO("Moving Chest");

    ChestControlInterface chestTraj(nh);
    float roll = 0*M_PI/180; 
    float pitch = 30*M_PI/180;
    float yaw = 50*M_PI/180;
    float duration = 5.0f;

    chestTraj.controlChest(roll, pitch, yaw, duration);

    ros::Duration(duration).sleep();
    ROS_INFO("Motion Finished");

    return 0;
}