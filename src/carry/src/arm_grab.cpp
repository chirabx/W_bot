#include <ros/ros.h>
#include "../include/mini2_arm/mini2_arm.h"
#include <apriltags2_ros/AprilTagDetectionArray.h>

Mini2_ARM arm;
int tag_id = 0;
int tag = 1;
bool grab = true;
bool grab_finished = false;

void tag_callback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    
        if (grab)
     {
        // arm.armSetZeroCal();
        // ros::Duration(10.0).sleep();
        ROS_INFO("Move to grab pose!!!!!");
        int init_pos[3] = {0, 6000, 0};
        arm.armSetAbsSteps(init_pos);

        ros::Duration(1.0).sleep(); //3.0
        arm.armSetPump(true);//开气泵
        
        int init_1_pos[3] = {0, 3650, -3500}; //2800,-3000
        arm.armSetAbsSteps(init_1_pos);

        ros::Duration(3.0).sleep();
        ROS_INFO("Grab woods!!!!!!");

        arm.armSetValve(false);//关气阀
        //ros::Duration(1.0).sleep();

        // arm.armSetPump(false);
        // ros::Duration(2.0).sleep();

        ROS_INFO("Move to end pos!!!!!!");
        int end_pos[3] = {0, 6000, 0};
        arm.armSetAbsSteps(end_pos);
        ros::Duration(2.0).sleep();  
        
        grab = false;
        grab_finished = true;
    }
}

int main(int argc, char** argv) {

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

    nh.getParam("arm_grab_node/tag", tag);

    ros::Duration(1.0).sleep(); // 等待初始化完成
    ros::Subscriber tag_sub = nh.subscribe("/tag_detections", 1000, tag_callback);

    ros::Duration(1.0).sleep(); // 等待订阅完成
    while (ros::ok() && !grab_finished)
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep(); 
    }
    ros::shutdown();
    return 0;
}