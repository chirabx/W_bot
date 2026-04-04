#include <ros/ros.h>
#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h> // 用于发布整数消息
#include <tf/tf.h>
#include <stdio.h>

ros::Publisher pub;
ros::Publisher tag_id_pub; // 用于发布 tag_id 的发布者
bool tag_1_detected = false;   // 标志变量，表示是否检测到过 ID 为 1 的 AprilTag
bool tag_2_detected = false;   // 标志变量，表示是否检测到过 ID 为 2 的 AprilTag
bool try_again = true;
int tag_id = 0;

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
            // ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        vel_msg.linear.y = 0.0;
        pub.publish(vel_msg);
}

void Turn_safe(ros::Publisher& pub, double angular_z, double distance)
{
    geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = angular_z;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < distance)
        {
            pub.publish(vel_msg);
            // ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.angular.z = 0.0;
        pub.publish(vel_msg);
}

void callback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    size_t num_tags = msg->detections.size(); // 检测到的 AprilTag 数量
    if (num_tags == 0) // 没有检测到任何 AprilTag
    {
        ROS_INFO("No AprilTags detected.");
        Move_safe(pub, -0.06, 0, 3);
    }
    
    else if(num_tags == 2)
    {
        Move_safe(pub, 0, 0.04, 3);
    }
    else
    {
        for (const auto& detection : msg->detections)// 遍历检测到的 AprilTag
        {
            tag_id = detection.id[0];  // AprilTag 的 ID 是一个整数数组，通常取第一个元素
            ROS_INFO("Detected AprilTag ID: %d", tag_id); // 输出 AprilTag 的 ID

            const auto& pose = detection.pose; // 获取 AprilTag 的位姿
            double x = pose.pose.pose.position.x * 1000;
            double y = pose.pose.pose.position.y * 1000;
            double z = pose.pose.pose.position.z * 1000;

            const auto& quaternion = detection.pose.pose.pose.orientation;
            // 将四元数转换为欧拉角
            tf::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // 转换为度
            roll = roll * 180.0 / M_PI;
            pitch = pitch * 180.0 / M_PI;
            yaw = yaw * 180.0 / M_PI;

            // 打印位置和角度信息
            ROS_INFO("x = %.2f, y = %.2f, z = %.2f", x, y, z);
            ROS_INFO("roll = %.2f, pitch = %.2f, yaw = %.2f", roll, pitch, yaw);

            //倾斜角调节
            if (roll <=-50 && roll>=-130) 
            {
                Turn_safe(pub, 0.07, 6);//zuo
            }
            else if (roll>=50&&roll<=130)
            {
                Turn_safe(pub, -0.07, 6);//you
            }
            else //倾斜角正常
            {
                // 发布 tag_id
                std_msgs::Int32 tag_id_msg;
                tag_id_msg.data = tag_id;
                tag_id_pub.publish(tag_id_msg);
                tag_id_pub.publish(tag_id_msg);
                tag_id_pub.publish(tag_id_msg);

                // 左右调节
                if (x < -8 ) //
                {
                    Move_safe(pub, 0, -0.05, 2);
                }
                else if (x > 8)
                {
                    Move_safe(pub, 0, 0.05, 2);   
                }
                else //左右正常
                {
                    // 远近调节
                    if (z > 70)
                    {
                        Move_safe(pub, 0.05, 0, 2);
                    }
                    else if (z < 63)
                    {
                        Move_safe(pub, -0.05, 0, 2);
                    }
                    else //远近正常
                    {
                        try_again = false;  // 设置标志变量
                        ROS_INFO("Conditions met. Setting try_again to false.");
                    }
                }
            }
        }
    }

    if (try_again == false) // 完成所有调节，实现抓取
    {
        if (tag_id == 1)
        {
            system("roslaunch carry arm_grab_1.launch");
            ros::shutdown(); // 关闭当前节点
        }
        else if (tag_id == 2)
        {
            system("roslaunch carry arm_grab_2.launch");
            ros::shutdown(); // 关闭当前节点
        }
    }
}

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "apriltag_id_listener");
    ros::NodeHandle nh;

    // 创建一个发布者，用于发布机器人速度消息
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 创建一个发布者，用于发布 tag_id 消息
    tag_id_pub = nh.advertise<std_msgs::Int32>("/detected_tag_id", 10);

    // 订阅 /tag_detections 主题
    ros::Subscriber sub = nh.subscribe("/tag_detections", 1, callback);
    ros::spin();

    return 0;
}

