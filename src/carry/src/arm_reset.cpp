#include <ros/ros.h>
#include "../include/mini2_arm/mini2_arm.h"

Mini2_ARM arm;

// void try_loose()
// {
//     ROS_INFO("Release Woods!!!!!");
//     int release_pos[3] = {-7600, 1580, -4800};
//     arm.armSetAbsSteps(release_pos);
//     ros::Duration(2.0).sleep();
//     int release_pos1[3] = {0, 6000, 0};
//     arm.armSetAbsSteps(release_pos1);
//     ros::Duration(2.0).sleep();
//     arm.armSetValve(true);
//     arm.armSetPump(false);
//     ros::Duration(2.0).sleep();
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_put_node");
    ros::NodeHandle nh;
    arm.armSetValve(true);//打开气阀（松开夹爪）
    arm.armSetPump(false);//关闭气泵
    // 初始化机械臂
    char portname[20];
    sprintf(portname, "/dev/arm");
    arm.init(portname, 115200);

    ROS_INFO("Starting arm reset process...");
    // try_loose();
    // 进行机械臂复位操作
    arm.armSetZeroCal();//执行机械臂零点校准    
    ROS_INFO("Arm reset completed.");
    // 进行机械臂复位操作
    // 可以根据需要添加一些延时，确保复位操作完成
    // ros::Duration(10.0).sleep();
    return 0;
}
