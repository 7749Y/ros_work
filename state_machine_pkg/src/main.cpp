#include "state_machine_pkg/state_machine.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "state_machine_node");
    ros::NodeHandle nh("~");

    ROS_INFO("========================================");
    ROS_INFO("  2026 睿抗机器人开发者大赛");
    ROS_INFO("  魔力元宝 - 工业组省赛任务一");
    ROS_INFO("  基于AR码的仓储机器人开发");
    ROS_INFO("========================================");

    StateMachine sm(nh);
    State final_state = sm.run();

    if (final_state == State::Finished)
    {
        ROS_INFO("任务成功完成!");
        return 0;
    }
    else if (final_state == State::Error)
    {
        ROS_ERROR("任务失败，请检查日志后重启");
        return 1;
    }
    else
    {
        ROS_WARN("状态机异常退出: %s", stateToString(final_state).c_str());
        return 2;
    }
}
