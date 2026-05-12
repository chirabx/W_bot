#include <ros/ros.h>
#include "../include/mini2_arm/mini2_arm.h"
#include <apriltags2_ros/AprilTagDetectionArray.h>

Mini2_ARM arm;

void try_loose()
{
    ROS_INFO("Release Woods!!!!!");

    int release_pos[3] = {0, 0, -6000};
    arm.armSetAbsSteps(release_pos);
    ros::Duration(3.0).sleep();
    arm.armSetValve(true);
    arm.armSetPump(false);
    
    ros::Duration(2.0).sleep();
    int release_pos_1[3] = {0, 6000, 0};
    arm.armSetAbsSteps(release_pos_1);
    //ros::Duration(2.0).sleep();

    //ros::Duration(10.0).sleep();//2
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_put_node");
    ros::NodeHandle nh;
   
    // 初始化机械臂
    char portname[20];
    sprintf(portname, "/dev/arm");
    arm.init(portname, 115200);

    //ROS_INFO("Starting arm reset process...");

    try_loose();
    
    // 可以根据需要添加一些延时，确保复位操作完成
    //ros::Duration(10.0).sleep();
    // 进行机械臂复位操作11111
    // arm.armSetZeroCal();
    // ros::Duration(10.0).sleep();
    // ROS_INFO("Arm reset completed.");
    
    ros::shutdown();
    return 0;
}