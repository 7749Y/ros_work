#include <ros/ros.h>
#include "std_srvs/Empty.h"
#include "arm_controller/move.h"

// 创建服务客户端
ros::ServiceClient armmove_client;
ros::ServiceClient pick_client;
ros::ServiceClient put_client;

bool arm_move(float x,float y,float z)
{
    // 等待服务上线
    ROS_INFO("等待服务 /goto_position 启动...");
    armmove_client.waitForExistence();
    ROS_INFO("服务已连接！"); 
    // 定义服务消息
    arm_controller::move srv;

    // 填充请求数据
    srv.request.pose.position.x = x;
    srv.request.pose.position.y = y;
    srv.request.pose.position.z = z;

    armmove_client.call(srv);
    return srv.response.success;
}

void set_pump(bool state)
{
    // 等待服务上线
    ROS_INFO("等待抓取服务启动...");
    pick_client.waitForExistence();
    put_client.waitForExistence();
    ROS_INFO("服务已连接！"); 
    // 定义服务消息
    std_srvs::Empty srv;
    if (state)
    {
        pick_client.call(srv);
    }
    else
    {
        put_client.call(srv);
    }
}

int main(int argc, char** argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "arm_control_node");
    ros::NodeHandle nh;
    armmove_client = nh.serviceClient<arm_controller::move>("/goto_position");
    pick_client = nh.serviceClient<std_srvs::Empty>("/swiftpro/on");
    put_client = nh.serviceClient<std_srvs::Empty>("/swiftpro/off");

    // 进入安全点
    arm_move(150,0,120);
    // 入刀点
    arm_move(107,185,42+10);
    // 抓取点
    arm_move(107,185,42);
    // 开启吸盘
    ros::Duration(1.0).sleep();
    set_pump(true);
    // 出刀点
    arm_move(107,185,42+10);

    // 进入安全点
    arm_move(150,0,120);
    // 入刀点2
    arm_move(107,128,42+10);
    // 放置点
    arm_move(107,128,45);
    // 关闭吸盘
    ros::Duration(1.0).sleep();
    set_pump(false);
    // 出刀点2
    arm_move(107,128,42+10);

    // 回到安全点
    arm_move(150,0,120);
    return 0;
}