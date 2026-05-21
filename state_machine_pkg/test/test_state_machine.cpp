#include "state_machine_pkg/state_machine.h"
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <boost/thread.hpp>

// ============================================================
// 工业组省赛任务一 状态机测试文件
// 测试方式（无需实际硬件）：
//   1. 启动 roscore
//   2. 运行测试节点：rosrun state_machine_pkg test_state_machine_node
//   3. 节点会自动发布开始信号并模拟完整流程
// ============================================================

int main(int argc, char** argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "test_state_machine");
    ros::NodeHandle nh;

    ROS_INFO("========================================");
    ROS_INFO("  状态机测试程序");
    ROS_INFO("  2026 魔力元宝 - 工业组省赛任务一");
    ROS_INFO("  模式: 模拟测试 (无需硬件)");
    ROS_INFO("========================================");

    // 创建状态机
    StateMachine sm(nh);

    // 延迟发布开始信号，让状态机先进入 WaitStart 状态
    ROS_INFO("[Test] 2秒后将自动发布比赛开始信号...");
    ros::Duration(1.0).sleep();

    // 在独立线程中发布开始信号
    boost::thread start_signal_thread([&nh]() {
        ros::Duration(1.0).sleep();  // 等状态机进入 WaitStart
        ROS_INFO("[Test] >>> 发布比赛开始信号到 /competition/start <<<");
        ros::Publisher start_pub = nh.advertise<std_msgs::Empty>("/competition/start", 1);
        ros::Duration(0.5).sleep();
        std_msgs::Empty msg;
        start_pub.publish(msg);
        ROS_INFO("[Test] 开始信号已发送");
    });

    // 运行状态机
    ROS_INFO("[Test] 启动状态机...");
    ros::Time test_start = ros::Time::now();
    State final_state = sm.run();
    ros::Time test_end = ros::Time::now();

    // 等待线程结束
    if (start_signal_thread.joinable()) {
        start_signal_thread.join();
    }

    // 输出测试结果
    ROS_INFO("========================================");
    ROS_INFO("  测试结果");
    ROS_INFO("========================================");
    ROS_INFO("  总耗时: %.1f 秒", (test_end - test_start).toSec());
    ROS_INFO("  最终状态: %s", stateToString(final_state).c_str());

    if (final_state == State::Finished)
    {
        ROS_INFO("  ★ 测试通过: 状态机完整执行了任务一流程 ★");
        ROS_INFO("  流程: Init → WaitStart → NavToPickPoint1 → RecognizeAR1");
        ROS_INFO("        → Grasp1 → Store1 → NavToPickPoint2 → RecognizeAR2");
        ROS_INFO("        → Grasp2 → Store2 → NavToDeparture → WaitUnload");
        ROS_INFO("        → NavToCharge → DoCharge → NavToDepartureFinal → Finished");
        return 0;
    }
    else if (final_state == State::Error)
    {
        ROS_WARN("  × 测试未通过: 状态机进入了错误状态");
        ROS_WARN("  提示: 确保 roscore 正在运行");
        return 1;
    }
    else
    {
        ROS_WARN("  ? 测试结果不确定: 最终状态为 %s", stateToString(final_state).c_str());
        return 2;
    }
}
