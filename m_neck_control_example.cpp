#include <tough_controller_interface/head_control_interface.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nech_control");
    ros::NodeHandle nh;
    ROS_INFO("Moving Neck");

    HeadControlInterface neckTraj(nh);

    float value1 = 0;
    float value2 = 1;
    float value3 = 0;
    
    const std::vector<std::vector<float>>& neck_joints = {{value1, value2, value3}};
    neckTraj.moveNeckJoints(neck_joints, 2.0f);


    ros::Duration(2).sleep();
    return 0;

}