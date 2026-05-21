#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ar_pose/Track.h>
#include <arm_controller/move.h>
#include <relative_move/SetRelativeMove.h>
#include <string>
#include <functional>

// ============================================================
// 工业组省赛任务一 - 状态枚举
// 对应规则：取货点一 → 取货点二 → 货物出库 → 自主回充 → 返回出发区
// ============================================================
enum class State {
    Init,                // 初始化：加载参数、连接服务、初始化硬件
    WaitStart,           // 等待比赛开始信号
    NavToPickPoint1,     // 导航到1号加工中心卸货区
    RecognizeAR1,        // 识别1号位AR码，获取物块位姿
    Grasp1,              // 抓取1号位物块
    Store1,              // 将物块放入中转箱
    NavToPickPoint2,     // 导航到2号加工中心卸货区
    RecognizeAR2,        // 识别2号位AR码
    Grasp2,              // 抓取2号位物块
    Store2,              // 将物块放入中转箱
    NavToDeparture,      // 导航到出发区（货物出库）
    WaitUnload,          // 货物出库动作
    NavToCharge,         // 导航到充电桩区域
    DoCharge,            // 对接充电
    NavToDepartureFinal, // 充电完成，返回出发区待机
    Error,               // 错误状态：停止动作，等待人工干预
    Finished             // 任务完成
};

// 状态名称映射，用于日志输出
inline std::string stateToString(State s) {
    switch (s) {
        case State::Init:                return "Init";
        case State::WaitStart:           return "WaitStart";
        case State::NavToPickPoint1:     return "NavToPickPoint1";
        case State::RecognizeAR1:        return "RecognizeAR1";
        case State::Grasp1:              return "Grasp1";
        case State::Store1:              return "Store1";
        case State::NavToPickPoint2:     return "NavToPickPoint2";
        case State::RecognizeAR2:        return "RecognizeAR2";
        case State::Grasp2:              return "Grasp2";
        case State::Store2:              return "Store2";
        case State::NavToDeparture:      return "NavToDeparture";
        case State::WaitUnload:          return "WaitUnload";
        case State::NavToCharge:         return "NavToCharge";
        case State::DoCharge:            return "DoCharge";
        case State::NavToDepartureFinal: return "NavToDepartureFinal";
        case State::Error:               return "Error";
        case State::Finished:            return "Finished";
        default:                         return "Unknown";
    }
}

// ============================================================
// 状态机类
// ============================================================
class StateMachine {
public:
    StateMachine(ros::NodeHandle& nh);
    ~StateMachine();

    // 运行状态机（主循环），返回最终状态
    State run();

    // 获取当前状态
    State getCurrentState() const { return current_state_; }

    // === 占位 / 实际功能函数 ===
    // 每个函数返回 true 表示成功，false 表示失败

    // Init: 初始化机器人各个模块
    bool initialize();

    // WaitStart: 等待比赛开始信号（话题或按键）
    bool waitForStartSignal();

    // NavToPickPoint: 导航到指定加工中心的卸货区
    bool navigateTo(double x, double y, double z, double w, const std::string& name);

    // RecognizeAR: 识别指定ID的AR码，获取位姿
    bool recognizeAR(int ar_id, float dist);

    // Grasp: 执行抓取动作，传入目标坐标
    bool grasp(float x, float y, float z);

    // Store: 将物块放置到机器人中转箱
    bool store();

    // WaitUnload: 货物出库动作（放置到出发区指定区域）
    bool unloadGoods();

    // DoCharge: 对接充电
    bool startCharging();

    // 进入错误状态，停止所有动作
    void enterError(const std::string& reason);

    // 检查ROS是否正常
    bool ok() const { return ros::ok(); }

private:
    // 状态转移：执行当前状态的动作，返回下一状态
    State processState(State s);

    // 检查是否需要超时退出
    bool isTimeout(ros::Time start, double timeout_sec);

private:
    ros::NodeHandle& nh_;
    State current_state_;
    ros::Rate rate_;

    // 超时设置（秒）
    double timeout_wait_start_;
    double timeout_nav_;
    double timeout_recognize_;
    double timeout_grasp_;
    double timeout_charge_;

    // 比赛参数
    int ar_id_1_;       // 1号位AR码ID（规则：ar-1或ar-2，位于4号加工中心）
    int ar_id_2_;       // 2号位AR码ID（规则：ar-3或ar-4，位于5号加工中心）
    float ar_dist_;     // AR识别距离

    // 导航目标点坐标（可配置）
    double pick_point_1_x_, pick_point_1_y_;
    double pick_point_2_x_, pick_point_2_y_;
    double departure_x_, departure_y_;
    double charge_x_, charge_y_;

    // 开始信号标志
    bool start_signal_received_;
    ros::Subscriber start_sub_;

    // 开始信号回调
    void startSignalCallback(const std_msgs::Empty::ConstPtr& msg);

    // === 服务客户端 ===
    ros::ServiceClient track_client_;       // /track (ar_pose::Track)
    ros::ServiceClient arm_move_client_;    // /goto_position (arm_controller::move)
    ros::ServiceClient pick_client_;        // /swiftpro/on (吸盘开启)
    ros::ServiceClient put_client_;         // /swiftpro/off (吸盘关闭)
    ros::ServiceClient relmove_client_;     // /relative_move (relative_move::SetRelativeMove)

    // === Action 客户端 ===
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    MoveBaseClient* nav_client_;

    // === 当前抓取到的物块位姿（由AR识别填充）===
    float grasp_x_, grasp_y_, grasp_z_;
    bool grasp_pose_valid_;
};

#endif // STATE_MACHINE_H
