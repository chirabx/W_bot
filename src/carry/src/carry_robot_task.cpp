#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <iostream>
#include <cmath>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#include "../include/mini2_arm/mini2_arm.h"

// ======== 新增 TF 相关头文件 ========
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <memory>

Mini2_ARM arm;
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// 全局 TF 监听器指针
std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::shared_ptr<tf2_ros::TransformListener> tfListener;
ros::Publisher g_pub;
/*
 * @brief 移动到指定位置
 */

void Move_safe(ros::Publisher &pub, double linear_x, double linear_y, double distance)
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

void Move2goal(MoveBaseClient &ac, double x, double y, double yaw)
{
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal! Target: (%.3f, %.3f, %.3f)", x, y, yaw);
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Navigation succeeded! Starting position fine-tuning...");

        // ========== 定位微调部分 ==========
        if (tfBuffer)
        {
            try
            {
                // 1. 获取实际位姿
                geometry_msgs::TransformStamped tfStamped = tfBuffer->lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));

                double actual_x = tfStamped.transform.translation.x;
                double actual_y = tfStamped.transform.translation.y;
                double actual_yaw = tf2::getYaw(tfStamped.transform.rotation);

                ROS_INFO("Actual Pose - X: %.3f, Y: %.3f, Yaw: %.3f", actual_x, actual_y, actual_yaw);

                // 2. 计算全局偏差
                double delta_x = x - actual_x;
                double delta_y = y - actual_y;
                ROS_INFO("Global Error - dX: %.3f, dY: %.3f", delta_x, delta_y);

                // 3. 转换到机器人局部坐标系
                double local_dx = delta_x * cos(actual_yaw) + delta_y * sin(actual_yaw);  // 前后方向
                double local_dy = -delta_x * sin(actual_yaw) + delta_y * cos(actual_yaw); // 左右方向

                ROS_INFO("Local Error - Forward: %.3f, Left: %.3f", local_dx, local_dy);

                // 4. 误差大于1cm时进行微调
                if (std::abs(local_dx) > 0.01 || std::abs(local_dy) > 0.01)
                {
                    ROS_INFO("Error > 1cm, starting fine-tuning...");

                    // 前后方向微调
                    if (std::abs(local_dx) > 0.01)
                    {
                        double adjust_vx = (local_dx > 0) ? 0.05 : -0.05;
                        // 计算需要的步数：步数 = 距离 / (速度 × 0.1)
                        int step_count = std::min(50, (int)(std::abs(local_dx) / 0.05 * 10)); // 限制最大50步
                        ROS_INFO("Forward adjustment: %.3f m, steps: %d", local_dx, step_count);
                        Move_safe(g_pub, adjust_vx, 0.0, step_count);
                    }

                    // 左右方向微调
                    if (std::abs(local_dy) > 0.01)
                    {
                        double adjust_vy = (local_dy > 0) ? 0.05 : -0.05;
                        int step_count = std::min(50, (int)(std::abs(local_dy) / 0.05 * 10));
                        ROS_INFO("Lateral adjustment: %.3f m, steps: %d", local_dy, step_count);
                        Move_safe(g_pub, 0.0, adjust_vy, step_count);
                    }

                    ROS_INFO("Fine-tuning completed!");
                }
                else
                {
                    ROS_INFO("Error within 1cm, no adjustment needed.");
                }
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("Failed to get TF for actual pose, skipping adjustment: %s", ex.what());
            }
        }
        else
        {
            ROS_WARN("TF buffer not available, skipping adjustment!");
        }
        // ========== 微调结束 ==========
    }
    else
    {
        ROS_WARN("Navigation failed! Retry logic would go here...");
        // 可选：添加重试逻辑
    }
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "carry_robot_mini2");
    ros::NodeHandle nh;

    // ========== 初始化 TF 监听器 ==========
    tfBuffer = std::make_shared<tf2_ros::Buffer>();
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    g_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber tag_sub = nh.subscribe("/tag_detections", 1000, tagDetectionsCallback);
    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    double grab_desk_x = 0.0; // 桌子x坐标
    double grab_desk_y = 0.0; // 桌子y坐标

    double tag1_put_x = 0.0; // 物块1放置区x坐标
    double tag1_put_y = 0.0; // 物块1放置区y坐标
    double tag2_put_x = 0.0; // 物块2放置区x坐标
    double tag2_put_y = 0.0; // 物块2放置区y坐标

    double offset_left = 0.1;  // 左偏移量
    double offset_right = 0.1; // 右偏移量

    nh.getParam("carry_robot_task_node/grab_desk_x", grab_desk_x);
    nh.getParam("carry_robot_task_node/grab_desk_y", grab_desk_y);

    nh.getParam("carry_robot_task_node/tag1_put_x", tag1_put_x);
    nh.getParam("carry_robot_task_node/tag1_put_y", tag1_put_y);
    nh.getParam("carry_robot_task_node/tag2_put_x", tag2_put_x);
    nh.getParam("carry_robot_task_node/tag2_put_y", tag2_put_y);

    nh.getParam("carry_robot_task_node/offset_length", offset_left);
    nh.getParam("carry_robot_task_node/offset_right", offset_right);

    char portname[20];
    sprintf(portname, "/dev/arm");
    if (!arm.init(portname, 115200))
    {
        ROS_ERROR("Failed to initialize arm at robot startup!");
        return 1; // 初始化失败则退出，防止后续盲目运行
    }

    ROS_INFO("Robot startup: Moving arm to initial position...");
    int release_pos[3] = {0, 6000, 0};
    arm.armSetAbsSteps(release_pos);

    // 强烈建议在此处加上延时（例如3~4秒），确保机械臂运动到位后再让底盘起步
    //ros::Duration(4.0).sleep();
    ROS_INFO("Arm reached initial position. Starting robot movement.");
    arm.close();

    // reset
    Move_safe(g_pub, 0.0, 0.3, 25);
    Move_safe(g_pub, 0.3, 0.0, 60);
    // 导航至桌子左侧前
    Move2goal(ac, 2.12, 0.05, -0.04); // jioa du=0 y=0.11 0.09
    system("roslaunch carry print_id.launch");

    ROS_INFO("%d", tagid);
    // 等待tag识别
    ros::Rate rate(10);
    while (ros::ok() && tagid == 0)
    {
        ros::spinOnce();
        rate.sleep();
    }
    if (tagid == 1) // 如果识别到为一号物块
    {
        ROS_INFO("Grab the tag1");
        Move_safe(g_pub, -0.1, 0.0, 30); // 后退30cm

        // 导航至一号物块放置区并放置
        Move2goal(ac, tag1_put_x, tag1_put_y, 1.57); // 1.57
        // Move_safe(pub, 0.0, 0.1, 10); // 左移10cm
        system("roslaunch carry arm_put.launch");
        // Move_safe(pub, 0.0, -0.1, 30); // 右移30cm
        // 导航至二号物块前并抓取
        Move2goal(ac, 2.12, 0.12, 0); // 2.12,0.13
        ROS_INFO("Grab the tag2");
        system("roslaunch carry print_id.launch");
        Move_safe(g_pub, -0.1, 0.0, 30); // 后退30cm
        // Move_safe(pub, 0, 0.1, 20); // 左移20cm

        // 导航至二号物块放置区并放置
        Move2goal(ac, tag2_put_x, tag2_put_y, 1.57);
        // Move_safe(pub, 0.0, 0.1, 10); // 左移10cm
        system("roslaunch carry arm_put.launch");
        // Move_safe(pub, 0.0, -0.1, 30); // 右移30cm
    }

    else if (tagid == 2) // 如果识别到为二号物块
    {
        ROS_INFO("Grab the tag2");
        Move_safe(g_pub, -0.1, 0.0, 30); // 后退30cm

        // 导航至二号物块放置区并放置
        Move2goal(ac, tag2_put_x, tag2_put_y, 1.57);
        // Move_safe(pub, 0.0, 0.1, 10); // 左移10cm
        system("roslaunch carry arm_put.launch");
        // Move_safe(pub, 0.0, -0.1, 30); // 右移30cm

        // 导航至一号物块前并抓取
        Move2goal(ac, 2.12, 0.10, 0); // y=0.13
        ROS_INFO("Grab the tag1");
        system("roslaunch carry print_id.launch");
        Move_safe(g_pub, -0.1, 0.0, 30); // 后退30cm
        // Move_safe(pub, 0, 0.1, 20); // 左移20cm

        // 导航至一号物块放置区并放置
        Move2goal(ac, tag1_put_x, tag1_put_y, 1.57);
        // Move_safe(pub, 0.0, 0.1, 10); // 左移10cm
        system("roslaunch carry arm_put.launch");
        // Move_safe(pub, 0.0, -0.1, 30); // 右移30cm
    }

    // 返回出发点
    Move2goal(ac, 0.05, 0.05, 0);
    ROS_INFO("Starting arm coordinate reset process to (0, 0, 0)...");
    if (arm.init(portname, 115200)) // 重新打开之前关闭的串口
    {
        //arm.armSetValve(true);  // 打开气阀（确保松开夹爪/气吸）
        //arm.armSetPump(false);  // 关闭气泵

        int reset_coords[3] = {0, 0, 0};
        arm.armSetAbsSteps(reset_coords); // 驱动机械臂移动到绝对坐标 (0, 0, 0)
        
        //ros::Duration(3.0).sleep();       // 延时等待机械臂移动到位
        arm.close();                      // 安全关闭串口
        ROS_INFO("Arm coordinate reset to (0,0,0) completed.");
    }
    else
    {
        ROS_ERROR("Failed to re-initialize arm for coordinate reset!");
    }
    Move_safe(g_pub, 0.0, -0.1, 30);
    Move_safe(g_pub, -0.1, 0.0, 30);
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Back !!!!");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }
}