#include <tough_controller_interface/pelvis_control_interface.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pelvis_control");
    ros::NodeHandle nh;
    ROS_INFO ("Moving pelvis to 1.2m height");

    PelvisControlInterface pelvisInt(nh);

    float height = 0.5f;
    float duration = 2.0f;
    pelvisInt.controlPelvisHeight(height,duration);

    ros::Duration(duration).sleep();
    ROS_INFO("Motion Finished");

    return 0;
}