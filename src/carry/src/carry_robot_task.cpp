#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <iostream>
#include<cmath>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/*
 * @brief 移动到指定位置
 */
void Move2goal(MoveBaseClient& ac, double x, double y,double yaw)
{
    tf2::Quaternion quaternion;
    quaternion.setRPY(0,0,yaw);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }
}
/*
 * @brief 到达目标后，移动指定距离。完成任务后再移动指定距离。可以避免碰撞。
 * @linear_x x方向速度，向前为正
 * @linear_y y方向速度，向左为正
 * @distance 移动距离
 */
void Move_safe(ros::Publisher& pub, double linear_x, double linear_y, double distance)
{
    geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = linear_x;
        vel_msg.linear.y = linear_y;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < distance)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        vel_msg.linear.y = 0.0;
        pub.publish(vel_msg);
}

int tagid = 0;


void tagDetectionsCallback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    for (const auto &detection : msg->detections)
    {
        if (detection.id.size() == 1 && detection.id[0] == 1)
        {
            tagid = 1;
        }
        else if (detection.id.size() == 1 && detection.id[0] == 2)
        {
            tagid = 2;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "carry_robot_mini2");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber tag_sub = nh.subscribe("/tag_detections", 1000, tagDetectionsCallback);
    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    double grab_desk_x = 0.0;    //桌子x坐标
    double grab_desk_y = 0.0;    //桌子y坐标

    double tag1_put_x = 0.0;    //物块1放置区x坐标
    double tag1_put_y = 0.0;    //物块1放置区y坐标
    double tag2_put_x = 0.0;    //物块2放置区x坐标
    double tag2_put_y = 0.0;    //物块2放置区y坐标

    double offset_left = 0.1; // 左偏移量
    double offset_right = 0.1; // 右偏移量


    nh.getParam("carry_robot_task_node/grab_desk_x", grab_desk_x);
    nh.getParam("carry_robot_task_node/grab_desk_y", grab_desk_y);

    nh.getParam("carry_robot_task_node/tag1_put_x", tag1_put_x);
    nh.getParam("carry_robot_task_node/tag1_put_y", tag1_put_y);
    nh.getParam("carry_robot_task_node/tag2_put_x", tag2_put_x);
    nh.getParam("carry_robot_task_node/tag2_put_y", tag2_put_y);

    nh.getParam("carry_robot_task_node/offset_length", offset_left);
    nh.getParam("carry_robot_task_node/offset_right", offset_right);

    //reset
    Move_safe(pub, 0.0, 0.3, 25);
    Move_safe(pub, 0.3, 0.0, 60); 
    //导航至桌子左侧前
    Move2goal(ac, 2.12,0.13,-0.04);//jioa du=0
    system("roslaunch carry print_id.launch");


    ROS_INFO("%d",tagid);
    //等待tag识别
    ros::Rate rate(10);
    while (ros::ok() && tagid == 0)
    {
        ros::spinOnce();
        rate.sleep();
    }
    if (tagid == 1) //如果识别到为一号物块
    {
        ROS_INFO("Grab the tag1");
        Move_safe(pub, -0.1, 0.0, 30); // 后退30cm
        
        //导航至一号物块放置区并放置
        Move2goal(ac, tag1_put_x, tag1_put_y,1.57);//1.57
        // Move_safe(pub, 0.0, 0.1, 10); // 左移10cm
        system("roslaunch carry arm_put.launch");
        // Move_safe(pub, 0.0, -0.1, 30); // 右移30cm

        //导航至二号物块前并抓取
        Move2goal(ac, 2.12,0.13,0);//2.12
        ROS_INFO("Grab the tag2");
        system("roslaunch carry print_id.launch");
        Move_safe(pub, -0.1, 0.0, 30); // 后退30cm
        // Move_safe(pub, 0, 0.1, 20); // 左移20cm

        //导航至二号物块放置区并放置
        Move2goal(ac, tag2_put_x, tag2_put_y,1.57);
        // Move_safe(pub, 0.0, 0.1, 10); // 左移10cm
        system("roslaunch carry arm_put.launch");
        // Move_safe(pub, 0.0, -0.1, 30); // 右移30cm

    }

    else if (tagid == 2) //如果识别到为二号物块
    {
        ROS_INFO("Grab the tag2");
        Move_safe(pub, -0.1, 0.0, 30); // 后退30cm

        //导航至二号物块放置区并放置
        Move2goal(ac, tag2_put_x, tag2_put_y,1.57);
        // Move_safe(pub, 0.0, 0.1, 10); // 左移10cm
        system("roslaunch carry arm_put.launch");
        // Move_safe(pub, 0.0, -0.1, 30); // 右移30cm

        //导航至一号物块前并抓取
        Move2goal(ac, 2.12,0.13,0);
        ROS_INFO("Grab the tag1");
        system("roslaunch carry print_id.launch");
        Move_safe(pub, -0.1, 0.0, 30); // 后退30cm
        // Move_safe(pub, 0, 0.1, 20); // 左移20cm

        //导航至一号物块放置区并放置
        Move2goal(ac, tag1_put_x, tag1_put_y,1.57);
        // Move_safe(pub, 0.0, 0.1, 10); // 左移10cm
        system("roslaunch carry arm_put.launch");
        // Move_safe(pub, 0.0, -0.1, 30); // 右移30cm
    }

    
    //返回出发点
    Move2goal(ac, 0.05, 0.05,0);
    Move_safe(pub, 0.0, -0.1, 20); 
    Move_safe(pub, -0.1, 0.0, 20); 
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Back !!!!");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }
}