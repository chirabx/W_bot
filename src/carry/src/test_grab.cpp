#include <ros/ros.h>
#include "../include/mini2_arm/mini2_arm.h"
#include <apriltags2_ros/AprilTagDetectionArray.h>

int main(int argc, char **argv)
{

    setlocale(LC_ALL, "");
    ros::init(argc, argv, "test_grab");
    ros::NodeHandle nh;
    
    Mini2_ARM arm;
    char portname[20];
    sprintf(portname, "/dev/arm");
     if (!arm.init(portname, 115200)) {
        ROS_ERROR("机械臂初始化失败，退出程序");
        return 1; // 初始化失败，退出程序并返回状态码1
    }
    ROS_INFO("Move to grab pose!!!!!");
        int init_pos[3] = {-200, 6000, 0};
        arm.armSetAbsSteps(init_pos);
        ros::Duration(3.0).sleep();
        int init_1_pos[3] = {-100, 3098, -3396};
        arm.armSetAbsSteps(init_1_pos);
        ros::Duration(3.0).sleep();
        ROS_INFO("Grab woods!!!!!!");
        arm.armSetPump(true);
        ros::Duration(2.0).sleep();
        arm.armSetValve(false);
        ros::Duration(2.0).sleep();
        // arm.armSetPump(false);
        // ros::Duration(2.0).sleep();
        ROS_INFO("Move to end pos!!!!!!");
        int end_pos[3] = {0, 6000, 0};
        arm.armSetAbsSteps(end_pos);
        ros::Duration(3.0).sleep();

    return 0;
}
