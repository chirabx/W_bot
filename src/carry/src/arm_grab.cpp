#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <geometry_msgs/Twist.h> //yyx
#include <std_msgs/Int32.h>

#include "upros_message/ArmPosition.h"
#include "std_srvs/Empty.h"
#include <ros/ros.h>
#include <tf/tf.h>

void sleep(double second)
{
    ros::Duration(second).sleep();
}

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
}//gly


int main(int argc, char **argv)
{

    ros::init(argc, argv, "mgrab_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10); // yyx

    // 1. 先用 int 类型读取 launch 文件里的参数（因为 value="1" 会被当成 int）
    int tag_id = 1; // 默认给个1
    if (!private_nh.getParam("tag", tag_id)) {
        ROS_WARN("Failed to get param 'tag', using default: %d", tag_id);
    }
    
    // 2. 拼接出真实的 TF 坐标系名称 (通常 apriltag 生成的坐标系叫 "tag_1", "tag_2")
    std::string tag_link = "tag_" + std::to_string(tag_id);
    ROS_INFO("The value of tag is %s", tag_link.c_str());

    // 初始化service创建服务客户端
    ros::ServiceClient arm_pose_client = nh.serviceClient<upros_message::ArmPosition>("/control_center/arm_pos_service");
    ros::ServiceClient arm_grab_client = nh.serviceClient<std_srvs::Empty>("/control_center/grab_service");
    if (!arm_pose_client.waitForExistence(ros::Duration(10.0)))
    {
        ROS_ERROR("Service not available: /control_center/arm_pos_service");
        return 1;
    }
    if (!arm_grab_client.waitForExistence(ros::Duration(10.0)))
    {
        ROS_ERROR("Service not available: /control_center/grab_service");
        return 1;
    }
    ROS_INFO("Services ready: /control_center/arm_pos_service, /control_center/grab_service");

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ROS_INFO("tf coordinate transformaing....");

    // 3. 带异常保护的 TF 变换查询
    geometry_msgs::TransformStamped tfs_1;
    bool use_base_link_fallback = false;
    int arm_base_forward_offset_mm = 230;
    private_nh.param("arm_base_forward_offset_mm", arm_base_forward_offset_mm, 230);
    try
    {
        // 优先直接获取 tag_link 到 arm_base_link 的坐标变换
        tfs_1 = buffer.lookupTransform("arm_base_link", tag_link, ros::Time(0), ros::Duration(5.0));
        ROS_INFO("Using TF source: arm_base_link -> %s", tag_link.c_str());
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("TF arm_base_link->%s failed: %s, fallback to base_link", tag_link.c_str(), ex.what());
        try
        {
            tfs_1 = buffer.lookupTransform("base_link", tag_link, ros::Time(0), ros::Duration(5.0));
            use_base_link_fallback = true;
            ROS_INFO("Using TF source: base_link -> %s", tag_link.c_str());
        }
        catch (tf2::TransformException &ex2)
        {
            ROS_ERROR("TF fallback base_link->%s failed: %s", tag_link.c_str(), ex2.what());
            ros::shutdown();
            return 1;
        }
    }
    
    int bias_x = 0;  // x方向的偏移，增加的机械臂往左多探的毫米数
    int bias_y = 95; // y方向的偏移，增加的机械臂往前多探的毫米数
    int bias_z = 75; // z方向的偏移，增加的机械臂往上多探的毫米数

    // 单位转换，ros坐标系到逆运算坐标系
    int y1 = int(tfs_1.transform.translation.x * 1000.0);//原为以底盘为坐标系的x坐标
    ROS_INFO("Raw TF(mm): tx=%d ty=%d tz=%d",
             int(tfs_1.transform.translation.x * 1000.0),
             int(tfs_1.transform.translation.y * 1000.0),
             int(tfs_1.transform.translation.z * 1000.0));
    if (use_base_link_fallback)
    {
        ROS_WARN("Applying base->arm forward offset: %d mm", arm_base_forward_offset_mm);
        y1 -= arm_base_forward_offset_mm;
    }
    int x = int(tfs_1.transform.translation.y * 1000.0);//原为以底盘为坐标系的y坐标
    int y = y1 + bias_y;
    int z = int(tfs_1.transform.translation.z * 1000.0) + bias_z;

    ROS_INFO("Initial target(mm): x=%d y=%d z=%d y1=%d (bias y=%d z=%d)", x, y, z, y1, bias_y, bias_z);

    std_srvs::Empty empty_srv;
    if (y1 < 135)
    {
        ROS_INFO("Base correction: too close, move back");
        Move_safe(pub,-0.08,0,6);
    }
    if (y1 > 145)
    {
        ROS_INFO("Base correction: too far, move forward for %d ticks", int((y1 - 145) / 2));
        Move_safe(pub,0.06,0,int((y1 - 145) / 2));
    }
    if (y1 > 160 && y1 < 180)
    {
        y = y - 30;
    }
    else if (y1 >= 180 && y1 < 200)
    {
        y = y - 70;
    }
    else if (y1 >= 200 && y1 < 220)
    {
        y = y - 90;
    }
    else if (y1 >= 220 && y1 < 240)
    {
        y = y - 110;
    }
    else if (y1 >= 240 && y1 < 260)
    {
        y = y - 130;
    }

    if (x >= -5)
    {
        if (x < 0)
        {
            x = -x;
        }
        ROS_INFO("Base correction: move left for %d ticks", x + 2);
        Move_safe(pub, 0, 0.04, x + 2);
        x = x + 10;
    }
    else if (x <= -25)
    {
        ROS_INFO("Base correction: move right for %d ticks", (-x) - 25 + 3);
        Move_safe(pub, 0, -0.05,(-x) - 25 + 3);
    }

    if (x > 30 || x < -40)
    {
        ROS_WARN("Target out of x range after correction, fallback to safe target");
        x = -16;
        y = 228;
        z = 82;
    }
    ROS_INFO("Final target(mm): x=%d y=%d z=%d", x, y, z);

    // 逆运算移动抓取到上方
    upros_message::ArmPosition srv;
    srv.request.x = -9;
    srv.request.y = 188;
    srv.request.z = 182;
    ROS_INFO("Call arm_pos_service pre-grab: x=%.1f y=%.1f z=%.1f", srv.request.x, srv.request.y, srv.request.z);

    bool ok = arm_pose_client.call(srv);
    ROS_INFO("arm_pos_service pre-grab result: call=%d status=%d", ok ? 1 : 0, int(srv.response.status));
    if (!ok || srv.response.status != 1)
    {
        ROS_ERROR("Failed to move arm to pre-grab pose");
        return 1;
    }

    sleep(3.0);

    ok = arm_grab_client.call(empty_srv);
    ROS_INFO("grab_service result: call=%d", ok ? 1 : 0);
    if (!ok)
    {
        ROS_ERROR("Failed to call grab service");
        return 1;
    }

    // 下探
    srv.request.x = x + 10;
    srv.request.y = y + 17; // 25
    srv.request.z = z - 7;  //-5
    ROS_INFO("Call arm_pos_service target: x=%.1f y=%.1f z=%.1f", srv.request.x, srv.request.y, srv.request.z);
    ok = arm_pose_client.call(srv);
    ROS_INFO("arm_pos_service target result: call=%d status=%d", ok ? 1 : 0, int(srv.response.status));
    if (!ok || srv.response.status != 1)
    {
        ROS_ERROR("Failed to move arm to target pose");
        return 1;
    }
    sleep(3.0);

    // 抬起来
    srv.request.x = 0;
    srv.request.y = 188; // 50
    srv.request.z = 182;
    ROS_INFO("Call arm_pos_service lift: x=%.1f y=%.1f z=%.1f", srv.request.x, srv.request.y, srv.request.z);
    ok = arm_pose_client.call(srv);
    ROS_INFO("arm_pos_service lift result: call=%d status=%d", ok ? 1 : 0, int(srv.response.status));
    if (!ok || srv.response.status != 1)
    {
        ROS_ERROR("Failed to lift arm after grab");
        return 1;
    }
    sleep(2.0);

    ros::shutdown();

    return 0;
}
