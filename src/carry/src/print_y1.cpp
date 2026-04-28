#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mgrab_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 读取 tag 编号，默认为 1
    int tag_id = 1;
    private_nh.getParam("tag", tag_id);
    std::string tag_link = "tag_" + std::to_string(tag_id);
    ROS_INFO("Monitoring TF: base_link -> %s", tag_link.c_str());

    // TF 监听器
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    // 偏移量（与原始抓取逻辑一致）
    const int bias_y = 95;
    const int bias_z = 75;

    ros::Rate rate(10);  // 10 Hz 输出频率

    while (ros::ok())
    {
        geometry_msgs::TransformStamped tfs;
        try
        {
            // 获取最新变换
            tfs = buffer.lookupTransform("base_link", tag_link, ros::Time(0), ros::Duration(0.5));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(5, "TF lookup failed: %s", ex.what());
            rate.sleep();
            continue;
        }

        // 单位转换（毫米）
        int y1 = static_cast<int>(tfs.transform.translation.x * 1000.0);
        int x  = static_cast<int>(tfs.transform.translation.y * 1000.0);
        int y  = y1 + bias_y;
        int z  = static_cast<int>(tfs.transform.translation.z * 1000.0) + bias_z;

        // 实时输出
        ROS_INFO("y1= %3d mm, x= %3d mm, y= %3d mm, z= %3d mm", y1, x, y, z);

        rate.sleep();
    }

    ros::shutdown();
    return 0;
}