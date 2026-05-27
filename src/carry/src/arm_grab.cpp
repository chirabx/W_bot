#include <ros/ros.h>
#include "../include/mini2_arm/mini2_arm.h"

Mini2_ARM arm;

void try_grab()
{
    ROS_INFO("Move to grab pose!!!!!");
    // int init_pos[3] = {0, 6000, 0};
    // arm.armSetAbsSteps(init_pos);

    // ros::Duration(3.0).sleep(); // 等待机械臂运动到位
    arm.armSetPump(true);       // 开气泵

    ros::Duration(1.0).sleep();
    
    int init_1_pos[3] = {0, 3650, -3500}; // 移动到抓取点
    arm.armSetAbsSteps(init_1_pos);

    ros::Duration(3.0).sleep();
    ROS_INFO("Grab woods!!!!!!");

    arm.armSetValve(false); // 关气阀，完成吸取

    ROS_INFO("Move to end pos!!!!!!");
    int end_pos[3] = {0, 6000, 0};
    arm.armSetAbsSteps(end_pos); // 抬起机械臂
    ros::Duration(2.0).sleep();
}

int main(int argc, char **argv)
{

    // 设置本地化以支持中文打印
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "arm_grab_node");
    ros::NodeHandle nh;

    // 初始化机械臂
    char portname[20];
    sprintf(portname, "/dev/arm");
    if (!arm.init(portname, 115200))
    {
        ROS_ERROR("机械臂初始化失败，退出程序");
        return 1; // 初始化失败，退出程序并返回状态码1
    }

    ros::Duration(1.0).sleep(); // 等待初始化完成

    // 直接执行抓取逻辑
    try_grab();

    // 抓取动作完成后，关闭节点退出
    ros::shutdown();
    return 0;
}