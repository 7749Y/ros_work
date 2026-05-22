#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

// 定义 Action 客户端类型
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient* nav_client;

bool navToGoal(double x, double y, double z, double w){
    ROS_INFO("等待连接 move_base 服务器...");
    nav_client->waitForServer();
    ROS_INFO("连接成功！");
    nav_client->cancelAllGoals();
    ROS_WARN("已清空所有导航任务！");

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = z;
    goal.target_pose.pose.orientation.w = w;

    ROS_INFO("发送导航目标...");
    nav_client->sendGoal(goal);

    ros::Rate rate(5);
    while (ros::ok())
    {
        actionlib::SimpleClientGoalState state = nav_client->getState();
        std::string state_str = state.toString();
        ROS_INFO("当前导航状态：%s", state_str.c_str());

        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("导航成功：已到达出发点！");
            return true;
        }
        else if (state == actionlib::SimpleClientGoalState::ABORTED)
        {
            ROS_ERROR("导航失败：无法到达目标！");
            return false;
        }
        else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
        {
            ROS_WARN("导航任务已被取消！");
            return false;
        }
        else if (state == actionlib::SimpleClientGoalState::REJECTED)
        {
            ROS_ERROR("导航目标被服务器拒绝！");
            return false;
        }
        else if (state == actionlib::SimpleClientGoalState::LOST)
        {
            ROS_ERROR("导航连接丢失！");
            return false;
        }
        rate.sleep();
    }
    return false;
}

int main(int argc, char** argv) {
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "auto_bkhome_node");
    ros::NodeHandle nh;

    nav_client = new MoveBaseClient("move_base", true);

    try {
        if (!navToGoal(0.003, -0.001, -0.012, 1.000)) {
            return -1;
        }
    } catch (ros::Exception& e) {
        ROS_ERROR("程序被中断：%s", e.what());
    }

    return 0;
}
