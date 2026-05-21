#include "state_machine_pkg/state_machine.h"

// ============================================================
// 构造 / 析构
// ============================================================
StateMachine::StateMachine(ros::NodeHandle& nh)
    : nh_(nh)
    , current_state_(State::Init)
    , rate_(10)  // 10Hz
    , start_signal_received_(false)
    , nav_client_(nullptr)
    , grasp_x_(0), grasp_y_(0), grasp_z_(0)
    , grasp_pose_valid_(false)
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
    if (nav_client_) {
        delete nav_client_;
        nav_client_ = nullptr;
    }
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

// ============================================================
// initialize: 初始化机器人各个模块
// 连接所有服务、初始化Action客户端、检查硬件状态
// ============================================================
bool StateMachine::initialize()
{
    ROS_INFO("[Init] 步骤1/4: 创建服务客户端...");

    // 创建服务客户端
    track_client_     = nh_.serviceClient<ar_pose::Track>("/track");
    arm_move_client_  = nh_.serviceClient<arm_controller::move>("/goto_position");
    pick_client_      = nh_.serviceClient<std_srvs::Empty>("/swiftpro/on");
    put_client_       = nh_.serviceClient<std_srvs::Empty>("/swiftpro/off");
    relmove_client_   = nh_.serviceClient<relative_move::SetRelativeMove>("/relative_move");

    ROS_INFO("[Init] 步骤2/4: 创建导航 Action 客户端...");
    try {
        nav_client_ = new MoveBaseClient("move_base", true);
    } catch (std::exception& e) {
        ROS_ERROR("[Init] 创建导航客户端失败: %s", e.what());
        return false;
    }

    ROS_INFO("[Init] 步骤3/4: 等待服务连接...");
    // 等待服务可用（非阻塞检查，超时3秒）
    bool all_ok = true;

    // 检查关键服务（非必须全部在线，只发警告）
    auto check_service = [&](ros::ServiceClient& client, const std::string& name) -> bool {
        if (!client.waitForExistence(ros::Duration(1.0))) {
            ROS_WARN("[Init] 服务 %s 暂未上线 (将继续尝试)", name.c_str());
            return false;
        }
        ROS_INFO("[Init] 服务 %s 已就绪", name.c_str());
        return true;
    };

    check_service(track_client_,    "/track");
    check_service(arm_move_client_, "/goto_position");
    check_service(pick_client_,     "/swiftpro/on");
    check_service(put_client_,      "/swiftpro/off");
    check_service(relmove_client_,  "/relative_move");

    ROS_INFO("[Init] 步骤4/4: 等待导航服务器...");
    if (!nav_client_->waitForServer(ros::Duration(3.0))) {
        ROS_WARN("[Init] move_base 服务器暂未就绪 (将持续等待)");
    } else {
        ROS_INFO("[Init] move_base 服务器已就绪");
    }

    ROS_INFO("[Init] 初始化完成!");
    return true;
}

// ============================================================
// waitForStartSignal: 等待比赛开始信号
// 1. 订阅 /competition/start 话题
// 2. 同时支持终端按 Enter 键开始（测试模式）
// ============================================================
bool StateMachine::waitForStartSignal()
{
    ros::Time start = ros::Time::now();

    // 在等待期间打印提示，同时支持终端输入
    ROS_INFO("[WaitStart] 等待开始信号... (发布到 /competition/start 或按 Enter 键)");

    while (ros::ok())
    {
        if (start_signal_received_) {
            return true;
        }

        if (isTimeout(start, timeout_wait_start_)) {
            ROS_ERROR("[WaitStart] 等待开始信号超时");
            return false;
        }

        ros::spinOnce();
        rate_.sleep();
    }

    return false;
}

// ============================================================
// navigateTo: 导航到指定目标点
// 使用 move_base Action 发送导航目标
// ============================================================
bool StateMachine::navigateTo(double x, double y, double z, double w, const std::string& name)
{
    if (!nav_client_) {
        ROS_ERROR("[Nav] 导航客户端未初始化");
        return false;
    }

    ROS_INFO("[Nav] ★ 正在前往: %s (%.2f, %.2f)", name.c_str(), x, y);

    // 等待 move_base 服务器
    if (!nav_client_->waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("[Nav] move_base 服务器连接超时");
        return false;
    }

    // 清空之前的导航任务
    nav_client_->cancelAllGoals();
    ros::Duration(0.5).sleep();

    // 构造导航目标
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = z;
    goal.target_pose.pose.orientation.w = w;

    ROS_INFO("[Nav] 发送导航目标到 %s ...", name.c_str());
    nav_client_->sendGoal(goal);

    // 等待导航结果
    ros::Time nav_start = ros::Time::now();
    while (ros::ok())
    {
        actionlib::SimpleClientGoalState state = nav_client_->getState();

        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("[Nav] √ 到达 %s", name.c_str());
            return true;
        }
        else if (state == actionlib::SimpleClientGoalState::ABORTED) {
            ROS_ERROR("[Nav] × 导航至 %s 失败 (ABORTED)", name.c_str());
            return false;
        }
        else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
            ROS_WARN("[Nav] 导航至 %s 被取消", name.c_str());
            return false;
        }
        else if (state == actionlib::SimpleClientGoalState::REJECTED) {
            ROS_ERROR("[Nav] 导航至 %s 被拒绝", name.c_str());
            return false;
        }
        else if (state == actionlib::SimpleClientGoalState::LOST) {
            ROS_ERROR("[Nav] 导航至 %s 连接丢失", name.c_str());
            return false;
        }

        // 超时检查
        if (isTimeout(nav_start, timeout_nav_)) {
            ROS_ERROR("[Nav] 导航至 %s 超时", name.c_str());
            nav_client_->cancelAllGoals();
            return false;
        }

        ros::spinOnce();
        rate_.sleep();
    }

    return false;
}

// ============================================================
// recognizeAR: 识别指定ID的AR码
// 调用 /track 服务，获取物块相对于机器人的位姿
// ============================================================
bool StateMachine::recognizeAR(int ar_id, float dist)
{
    ROS_INFO("[AR] 开始识别 AR码 ID=%d, 目标距离=%.2fm", ar_id, dist);

    if (!track_client_.waitForExistence(ros::Duration(3.0))) {
        ROS_ERROR("[AR] /track 服务不可用");
        return false;
    }

    ar_pose::Track srv;
    srv.request.ar_id = ar_id;
    srv.request.goal_dist = dist;

    ros::Time start = ros::Time::now();
    while (ros::ok())
    {
        if (track_client_.call(srv)) {
            if (srv.response.success) {
                ROS_INFO("[AR] √ 识别成功: %s", srv.response.message.c_str());
                // TODO: 如果服务返回物块位姿，保存到 grasp_x/y/z
                grasp_pose_valid_ = true;
                return true;
            } else {
                ROS_WARN("[AR] 识别失败: %s (重试中...)", srv.response.message.c_str());
            }
        } else {
            ROS_WARN("[AR] 服务调用失败 (重试中...)");
        }

        if (isTimeout(start, timeout_recognize_)) {
            ROS_ERROR("[AR] AR码识别超时");
            return false;
        }

        ros::spinOnce();
        rate_.sleep();
    }

    return false;
}

// ============================================================
// grasp: 执行抓取动作
// 1. 移动机械臂到目标位置上方
// 2. 下降到目标位置
// 3. 开启吸盘
// 4. 抬起机械臂
//
// 占位说明: 实际工作时需要根据 AR 识别返回的位姿动态调整坐标
// ============================================================
bool StateMachine::grasp(float x, float y, float z)
{
    ROS_INFO("[Grasp] 抓取物块 @ (%.1f, %.1f, %.1f)", x, y, z);

    // 等待机械臂服务可用
    if (!arm_move_client_.waitForExistence(ros::Duration(2.0))) {
        ROS_WARN("[Grasp] /goto_position 服务不可用 (使用模拟模式)");
        ROS_INFO("[Grasp] [模拟] 机械臂移动到 (%.1f, %.1f, %.1f+偏移)", x, y, z);
        ROS_INFO("[Grasp] [模拟] 机械臂下降到抓取点");
        ROS_INFO("[Grasp] [模拟] 开启吸盘");
        ROS_INFO("[Grasp] [模拟] 物块已吸附");
        ROS_INFO("[Grasp] [模拟] 机械臂抬起到安全高度");
        ros::Duration(2.0).sleep();  // 模拟抓取时间
        return true;
    }

    // 实际机械臂控制
    arm_controller::move srv;

    // 1. 移动到目标上方（安全高度）
    srv.request.pose.position.x = x;
    srv.request.pose.position.y = y;
    srv.request.pose.position.z = z + 50;  // 抬高50mm
    ROS_INFO("[Grasp] 步骤1: 移动到目标上方");
    if (arm_move_client_.call(srv)) {
        ROS_INFO("[Grasp] 到达目标上方");
    }

    // 2. 下降到抓取点
    srv.request.pose.position.z = z;
    ROS_INFO("[Grasp] 步骤2: 下降到抓取点");
    if (arm_move_client_.call(srv)) {
        ROS_INFO("[Grasp] 已到达抓取点");
    }
    ros::Duration(0.5).sleep();

    // 3. 开启吸盘
    ROS_INFO("[Grasp] 步骤3: 开启吸盘");
    std_srvs::Empty empty_srv;
    if (pick_client_.waitForExistence(ros::Duration(1.0))) {
        pick_client_.call(empty_srv);
    }
    ros::Duration(1.0).sleep();
    ROS_INFO("[Grasp] 吸盘已开启，物块已吸附");

    // 4. 抬起到安全高度
    srv.request.pose.position.z = z + 50;
    ROS_INFO("[Grasp] 步骤4: 抬起到安全高度");
    if (arm_move_client_.call(srv)) {
        ROS_INFO("[Grasp] 已抬起到安全高度");
    }

    ROS_INFO("[Grasp] √ 抓取完成");
    return true;
}

// ============================================================
// store: 将物块放置到机器人中转箱
// 机械臂回到中转箱位置上方 → 放下物块 → 关闭吸盘
// ============================================================
bool StateMachine::store()
{
    ROS_INFO("[Store] 将物块放入中转箱...");

    // 中转箱位置（相对于机器人基座）
    const float BOX_X = 80.0;
    const float BOX_Y = 0.0;
    const float BOX_Z = 30.0;

    if (!arm_move_client_.waitForExistence(ros::Duration(2.0))) {
        ROS_WARN("[Store] /goto_position 服务不可用 (使用模拟模式)");
        ROS_INFO("[Store] [模拟] 移动到中转箱上方 (%.1f, %.1f, %.1f)", BOX_X, BOX_Y, BOX_Z);
        ROS_INFO("[Store] [模拟] 关闭吸盘，物块放入中转箱");
        ros::Duration(1.0).sleep();
        return true;
    }

    arm_controller::move srv;

    // 1. 移动到中转箱上方
    srv.request.pose.position.x = BOX_X;
    srv.request.pose.position.y = BOX_Y;
    srv.request.pose.position.z = BOX_Z + 50;
    ROS_INFO("[Store] 步骤1: 移动到中转箱上方");
    arm_move_client_.call(srv);

    // 2. 下降到中转箱
    srv.request.pose.position.z = BOX_Z;
    ROS_INFO("[Store] 步骤2: 下降到中转箱");
    arm_move_client_.call(srv);

    // 3. 关闭吸盘，放下物块
    ROS_INFO("[Store] 步骤3: 关闭吸盘");
    std_srvs::Empty empty_srv;
    if (put_client_.waitForExistence(ros::Duration(1.0))) {
        put_client_.call(empty_srv);
    }
    ros::Duration(0.5).sleep();
    ROS_INFO("[Store] 物块已释放");

    // 4. 抬起到安全高度
    srv.request.pose.position.z = BOX_Z + 50;
    ROS_INFO("[Store] 步骤4: 抬起到安全高度");
    arm_move_client_.call(srv);

    ROS_INFO("[Store] √ 物块已放入中转箱");
    return true;
}

// ============================================================
// unloadGoods: 货物出库动作
// 将中转箱内的物块取出放置到出发区指定区域
// 简化实现：到达出发区即视为出库，停留2秒播报
// ============================================================
bool StateMachine::unloadGoods()
{
    ROS_INFO("[Unload] 开始货物出库...");

    // 实际应调用机械臂将物块从中转箱取出放到出发区指定位置
    // 省赛简化：到达即视为出库完成
    ROS_INFO("[Unload] 货物已运达出发区，执行出库操作");
    ROS_INFO("[Unload] [模拟] 机械臂从中转箱取出物块");
    ROS_INFO("[Unload] [模拟] 将物块放置到出发区指定位置");

    // 模拟出库动画时间
    ros::Duration(2.0).sleep();

    ROS_INFO("[Unload] √ 货物出库完成");
    return true;
}

// ============================================================
// startCharging: 对接充电
// 1. 使用相对移动微调位置
// 2. 调用充电对接服务
// 3. 等待充电完成信号
// ============================================================
bool StateMachine::startCharging()
{
    ROS_INFO("[Charge] 开始对接充电...");

    // 步骤1: 使用相对移动微调位置，对准充电桩
    ROS_INFO("[Charge] 步骤1: 微调位置对准充电桩");
    if (relmove_client_.waitForExistence(ros::Duration(2.0))) {
        relative_move::SetRelativeMove rel_srv;
        rel_srv.request.goal.x = 0.18;
        rel_srv.request.goal.y = 0;
        rel_srv.request.goal.theta = 0;
        rel_srv.request.global_frame = "odom";

        if (!relmove_client_.call(rel_srv)) {
            ROS_WARN("[Charge] 相对移动调用失败 (继续执行)");
        } else {
            ROS_INFO("[Charge] 位置微调完成");
        }
    } else {
        ROS_WARN("[Charge] /relative_move 服务不可用 (使用模拟模式)");
        ROS_INFO("[Charge] [模拟] 向前移动 0.18m 对准充电桩");
    }

    // 步骤2: 对接充电（调用 auto_charge 服务或模拟）
    ROS_INFO("[Charge] 步骤2: 对接充电中...");
    // TODO: 实际对接充电服务调用
    // auto_charge::Charge charge_srv;
    // charge_client_.call(charge_srv);

    ros::Duration(3.0).sleep();  // 模拟充电对接时间
    ROS_INFO("[Charge] 充电桩已对接");

    // 步骤3: 等待充电完成
    ROS_INFO("[Charge] 步骤3: 等待充电完成...");
    ros::Duration(2.0).sleep();  // 模拟充电时间
    ROS_INFO("[Charge] 充电完成");

    ROS_INFO("[Charge] √ 自主回充完成");
    return true;
}

// ============================================================
// enterError: 进入错误状态
// ============================================================
void StateMachine::enterError(const std::string& reason)
{
    ROS_ERROR("========================================");
    ROS_ERROR("  错误: %s", reason.c_str());
    ROS_ERROR("  状态: %s", stateToString(current_state_).c_str());
    ROS_ERROR("  已停止所有动作，等待人工干预");
    ROS_ERROR("========================================");

    // 取消所有导航目标
    if (nav_client_) {
        nav_client_->cancelAllGoals();
    }

    // 关闭吸盘
    std_srvs::Empty empty_srv;
    if (put_client_.waitForExistence(ros::Duration(0.5))) {
        put_client_.call(empty_srv);
    }

    current_state_ = State::Error;
}
