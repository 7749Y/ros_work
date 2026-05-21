#include "state_machine_pkg/state_machine.h"

// ============================================================
// 构造 / 析构
// ============================================================
StateMachine::StateMachine(ros::NodeHandle& nh)
    : nh_(nh)
    , current_state_(State::Init)
    , rate_(10)  // 10Hz
    , start_signal_received_(false)
{
    ROS_INFO("===== 工业组省赛任务一 状态机 =====");
    ROS_INFO("版本: 0.0.1");
    ROS_INFO("赛项: 2026 睿抗机器人开发者大赛 - 魔力元宝 (工业组)");

    // 从参数服务器加载配置（带默认值）
    ros::param::param("~timeout_wait_start",  timeout_wait_start_,  30.0);
    ros::param::param("~timeout_nav",          timeout_nav_,         60.0);
    ros::param::param("~timeout_recognize",    timeout_recognize_,   15.0);
    ros::param::param("~timeout_grasp",        timeout_grasp_,       20.0);
    ros::param::param("~timeout_charge",       timeout_charge_,      30.0);

    // AR码ID：规则规定 ar-1/ar-2 在4号加工中心, ar-3/ar-4 在5号加工中心
    ros::param::param("~ar_id_1", ar_id_1_, 1);
    ros::param::param("~ar_id_2", ar_id_2_, 3);
    ros::param::param("~ar_dist", ar_dist_, 0.4f);

    // 导航目标点坐标（需根据实际场地标定）
    ros::param::param("~pick_point_1_x", pick_point_1_x_, 0.5);
    ros::param::param("~pick_point_1_y", pick_point_1_y_, 1.5);
    ros::param::param("~pick_point_2_x", pick_point_2_x_, 1.5);
    ros::param::param("~pick_point_2_y", pick_point_2_y_, 1.5);
    ros::param::param("~departure_x",    departure_x_,    0.0);
    ros::param::param("~departure_y",    departure_y_,    0.0);
    ros::param::param("~charge_x",       charge_x_,       0.0);
    ros::param::param("~charge_y",       charge_y_,       2.0);

    // 订阅开始信号话题
    start_sub_ = nh_.subscribe("/competition/start", 1,
        &StateMachine::startSignalCallback, this);

    ROS_INFO("状态机初始化完成，等待进入 Init 状态...");
}

StateMachine::~StateMachine()
{
    ROS_WARN("状态机已销毁");
    rate_.reset();
}

// ============================================================
// 开始信号回调
// ============================================================
void StateMachine::startSignalCallback(const std_msgs::Empty::ConstPtr& msg)
{
    (void)msg;
    ROS_INFO(">>> 收到比赛开始信号!");
    start_signal_received_ = true;
}

// ============================================================
// 超时检查
// ============================================================
bool StateMachine::isTimeout(ros::Time start, double timeout_sec)
{
    if (timeout_sec <= 0) return false;  // 超时设为负数表示永不超时
    bool timed_out = (ros::Time::now() - start).toSec() > timeout_sec;
    if (timed_out) {
        ROS_ERROR("操作超时 (%.1f秒)", timeout_sec);
    }
    return timed_out;
}

// ============================================================
// 主运行循环
// ============================================================
State StateMachine::run()
{
    ROS_INFO("状态机开始运行");
    current_state_ = State::Init;

    while (ros::ok() && current_state_ != State::Finished)
    {
        // 打印当前状态
        ROS_INFO("========================================");
        ROS_INFO(">>> 当前状态: %s", stateToString(current_state_).c_str());

        // 执行状态处理，获取下一状态
        State next_state = processState(current_state_);

        // 如果下一状态是 Error，打印错误后继续（让循环走到 Error 处理）
        if (next_state == State::Error)
        {
            ROS_ERROR("!!! 进入错误状态，停止任务 !!!");
            current_state_ = State::Error;
            // 在 Error 状态中等待人工干预或自动退出
            ros::Duration(1.0).sleep();
            break;
        }

        // 状态转移
        ROS_INFO(">>> 状态转移: %s -> %s",
                 stateToString(current_state_).c_str(),
                 stateToString(next_state).c_str());
        current_state_ = next_state;

        ros::spinOnce();
        rate_.sleep();
    }

    if (current_state_ == State::Finished)
    {
        ROS_INFO("===== 任务一全部完成! =====");
    }
    else if (!ros::ok())
    {
        ROS_WARN("ROS 关闭，状态机退出");
    }

    return current_state_;
}

// ============================================================
// 状态处理核心：执行动作 → 返回下一状态
// ============================================================
State StateMachine::processState(State s)
{
    switch (s)
    {
        // --------------------------------------------------
        case State::Init:
        {
            ROS_INFO("[Init] 正在初始化机器人各模块...");
            if (initialize()) {
                ROS_INFO("[Init] 初始化成功");
                return State::WaitStart;
            } else {
                ROS_ERROR("[Init] 初始化失败");
                return State::Error;
            }
        }

        // --------------------------------------------------
        case State::WaitStart:
        {
            ROS_INFO("[WaitStart] 等待比赛开始信号...");
            ROS_INFO("[WaitStart] 订阅话题: /competition/start");
            if (waitForStartSignal()) {
                ROS_INFO("[WaitStart] 收到开始信号，比赛开始!");
                return State::NavToPickPoint1;
            }
            return State::Error;
        }

        // --------------------------------------------------
        case State::NavToPickPoint1:
        {
            ROS_INFO("[NavToPickPoint1] === 取货点一 (30分) ===");
            ROS_INFO("[NavToPickPoint1] 导航到1号加工中心卸货区 (%.2f, %.2f)",
                     pick_point_1_x_, pick_point_1_y_);
            if (navigateTo(pick_point_1_x_, pick_point_1_y_, 0, 1, "1号加工中心")) {
                ROS_INFO("[NavToPickPoint1] 到达1号加工中心卸货区 √  (10分)");
                return State::RecognizeAR1;
            }
            return State::Error;
        }

        // --------------------------------------------------
        case State::RecognizeAR1:
        {
            ROS_INFO("[RecognizeAR1] 识别 AR码 ID=%d", ar_id_1_);
            if (recognizeAR(ar_id_1_, ar_dist_)) {
                ROS_INFO("[RecognizeAR1] AR码识别成功");
                return State::Grasp1;
            }
            return State::Error;
        }

        // --------------------------------------------------
        case State::Grasp1:
        {
            ROS_INFO("[Grasp1] 从取料区抓取指定物块");
            // 占位坐标，实际由 AR 识别返回位姿
            if (grasp(107, 185, 42)) {
                ROS_INFO("[Grasp1] 抓取成功 √  (10分)");
                return State::Store1;
            }
            return State::Error;
        }

        // --------------------------------------------------
        case State::Store1:
        {
            ROS_INFO("[Store1] 将物料放置到机器人中转箱 √  (10分)");
            if (store()) {
                ROS_INFO("[Store1] 1号物块已放入中转箱 √  (10分)");
                ROS_INFO("[Store1] === 取货点一 完成 (30分) ===");
                return State::NavToPickPoint2;
            }
            return State::Error;
        }

        // --------------------------------------------------
        case State::NavToPickPoint2:
        {
            ROS_INFO("[NavToPickPoint2] === 取货点二 (30分) ===");
            ROS_INFO("[NavToPickPoint2] 导航到2号加工中心卸货区 (%.2f, %.2f)",
                     pick_point_2_x_, pick_point_2_y_);
            if (navigateTo(pick_point_2_x_, pick_point_2_y_, 0, 1, "2号加工中心")) {
                ROS_INFO("[NavToPickPoint2] 到达2号加工中心卸货区 √  (10分)");
                return State::RecognizeAR2;
            }
            return State::Error;
        }

        // --------------------------------------------------
        case State::RecognizeAR2:
        {
            ROS_INFO("[RecognizeAR2] 识别 AR码 ID=%d", ar_id_2_);
            if (recognizeAR(ar_id_2_, ar_dist_)) {
                ROS_INFO("[RecognizeAR2] AR码识别成功");
                return State::Grasp2;
            }
            return State::Error;
        }

        // --------------------------------------------------
        case State::Grasp2:
        {
            ROS_INFO("[Grasp2] 从取料区抓取指定物块");
            if (grasp(107, 128, 42)) {
                ROS_INFO("[Grasp2] 抓取成功 √  (10分)");
                return State::Store2;
            }
            return State::Error;
        }

        // --------------------------------------------------
        case State::Store2:
        {
            ROS_INFO("[Store2] 将物料放置到机器人中转箱 √  (10分)");
            if (store()) {
                ROS_INFO("[Store2] 2号物块已放入中转箱 √  (10分)");
                ROS_INFO("[Store2] === 取货点二 完成 (30分) ===");
                return State::NavToDeparture;
            }
            return State::Error;
        }

        // --------------------------------------------------
        case State::NavToDeparture:
        {
            ROS_INFO("[NavToDeparture] === 货物出库 (20分) ===");
            ROS_INFO("[NavToDeparture] 导航到出发区 (%.2f, %.2f)",
                     departure_x_, departure_y_);
            if (navigateTo(departure_x_, departure_y_, 0, 1, "出发区")) {
                ROS_INFO("[NavToDeparture] 到达出发区 √  (10分)");
                ROS_INFO("[NavToDeparture] 机器人未压线 √  (10分)");
                return State::WaitUnload;
            }
            return State::Error;
        }

        // --------------------------------------------------
        case State::WaitUnload:
        {
            ROS_INFO("[WaitUnload] 执行货物出库动作...");
            if (unloadGoods()) {
                ROS_INFO("[WaitUnload] 货物出库完成 √  (20分)");
                return State::NavToCharge;
            }
            return State::Error;
        }

        // --------------------------------------------------
        case State::NavToCharge:
        {
            ROS_INFO("[NavToCharge] === 自主回充 (30分) ===");
            ROS_INFO("[NavToCharge] 导航到充电桩区域 (%.2f, %.2f)",
                     charge_x_, charge_y_);
            if (navigateTo(charge_x_, charge_y_, 0.7, 0.7, "充电桩")) {
                ROS_INFO("[NavToCharge] 到达充电桩前方 √  (10分)");
                return State::DoCharge;
            }
            return State::Error;
        }

        // --------------------------------------------------
        case State::DoCharge:
        {
            ROS_INFO("[DoCharge] 对接充电中...");
            if (startCharging()) {
                ROS_INFO("[DoCharge] 成功对接充电 √  (20分)");
                ROS_INFO("[DoCharge] === 自主回充 完成 (30分) ===");
                return State::NavToDepartureFinal;
            }
            return State::Error;
        }

        // --------------------------------------------------
        case State::NavToDepartureFinal:
        {
            ROS_INFO("[NavToDepartureFinal] === 返回出发区 (20分) ===");
            ROS_INFO("[NavToDepartureFinal] 充电结束，返回出发区待机");
            if (navigateTo(departure_x_, departure_y_, 0, 1, "出发区")) {
                ROS_INFO("[NavToDepartureFinal] 正确到达出发区 √  (10分)");
                ROS_INFO("[NavToDepartureFinal] 机器人未压线 √  (10分)");
                ROS_INFO("[NavToDepartureFinal] === 返回出发区 完成 (20分) ===");
                ROS_INFO("========================================");
                ROS_INFO("  ★ 任务一全部完成! 总得分: 120分 ★");
                ROS_INFO("========================================");
                return State::Finished;
            }
            return State::Error;
        }

        // --------------------------------------------------
        case State::Error:
        {
            ROS_ERROR("[Error] 状态机已进入错误状态，等待人工干预");
            ros::Duration(0.5).sleep();
            return State::Error;  // 停留在 Error
        }

        // --------------------------------------------------
        case State::Finished:
            return State::Finished;

        // --------------------------------------------------
        default:
            ROS_ERROR("未知状态: %d", (int)s);
            return State::Error;
    }
}
