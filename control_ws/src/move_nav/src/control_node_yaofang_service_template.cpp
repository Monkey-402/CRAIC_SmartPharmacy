#include <memory>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cerrno>
#include <clocale>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <cv_bridge/cv_bridge.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

#include <sys/stat.h>
#include <sys/types.h>

#include "move_nav/Board1Decode.h"
#include "move_nav/Board2Decode.h"
#include "move_nav/CarLink.h"
#include "move_nav/JudgementReport.h"

/*
Board1Decode.srv：
*   bool has_a     # A 窗口是否有样本，有就置1
*   bool has_b     # B 窗口是否有样本
*   bool has_c     # C 窗口是否有样本
*   int32 delivery_slot   # 送达目标点 1=血常规，2=体液，3=免疫检测，4=激素检验
*   int32 sample_count     #样本数量
Board2Decode.srv：
*   string image_path  #图片路径
---
*   int32 wait_seconds #等待秒数
*   string speech_text #识别文字
*/

// 别名
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

enum VisionTask {
    NoVisionTask = 0,
    Board1Decode,
    Board2Decode
};

struct GoalTask {
    double x;
    double y;
    double yaw;
    std::string name;
};

// 任务点
// map 原点 = Gazebo spawn (0.271, -2.097, 0)；由旧图坐标换算
const std::vector<GoalTask> GOAL_LIST = {
    {0.0, 0.0, 0.0, "home"},
    {0.700, 0.0, 0.0, "board1_scan"},
    {0.501, 2.683, 2.225, "pickup_A"},
    {1.261, 3.199, 1.40, "pickup_B"},
    {1.381, 2.129, 1.57, "pickup_C"},
    {-0.500, 4.004, 3.082, "board2_scan"},
    {-1.946, 2.402, -1.57, "deliver_1"},
    {-1.100, 1.925, -1.57, "deliver_2"},
    {-1.860, 1.307, -2.359, "deliver_3"},
    {-1.115, 0.870, -1.339, "deliver_4"},
};

struct Board1Result {
    bool has_a = false;
    bool has_b = false;
    bool has_c = false;
    int delivery_slot = 1;// deliver_1 到 deliver_4
    int sample_count = 0;// 样本数量
};

struct Board2Result {
    int wait_seconds = 0;
    std::string speech_text;
};

ros::ServiceClient g_board1_client;
ros::ServiceClient g_board2_client;
std::string g_audio_dir = "audio";
std::string g_snapshot_dir = "/root/craic/control_ws/snapshots/";

static std::atomic<int> g_img_idx(0);// 图像序号计数器
static std::atomic<int> g_active_task(NoVisionTask);// 视觉服务开关

bool g_use_mock_data = false;
bool g_mock_navigation = false;
int g_max_rounds = 0;

// 启动时等待二维码和文字识别服务就绪的最长时间，OCR 首次加载通常会慢一些。
double g_vision_service_wait_timeout = 30.0;
// 启动时等待 move_base action server 就绪的最长时间。
double g_move_base_wait_timeout = 30.0;
// 单个导航目标发送后，等待目标进入 ACTIVE 或终态的最长时间。
double g_navigation_start_timeout = 30.0;

// 第几个导航点
size_t current_point = 0;

bool g_service_ok = false;// 服务是否成功返回
std::atomic<bool> g_snapshot_done(false);// 截图动作是否结束
std::atomic<bool> g_snapshot_ok(false);// 截图是否成功
std::mutex g_snapshot_image_path_mutex;
std::string g_snapshot_image_path;// 图片绝对路径
Board1Result g_board1_result;// 二维码缓存结果
Board2Result g_board2_result;// 文字缓存结果

std::mutex g_judgement_mutex;
double g_odom_x = 0.0;
double g_odom_y = 0.0;
double g_speed = 0.0;
std::string g_car_id = "1";
std::string g_current_task = "R";
std::string g_cv1 = "WAIT-0";
std::string g_cv2;
std::string g_default_cv1 = "WAIT-0";
std::string g_default_cv2;
std::string g_default_task = "R";
bool g_enable_judgement_report = true;
double g_judgement_report_rate = 1.5;
std::string g_judgement_report_topic = "/judgement/report";
ros::Publisher g_judgement_pub;
std::unique_ptr<tf2_ros::Buffer> g_tf_buffer;
std::unique_ptr<tf2_ros::TransformListener> g_tf_listener;
std::string g_map_frame = "map";
std::string g_base_frame = "base_link";
bool g_use_tf_pose = true;

// ---------- 双车协调与板一 slot 优先级 ----------
bool g_dual_car_mode = false;
std::string g_initial_station = "home";
std::string g_first_mover_car_id = "1";
double g_standby_x = -1.125;
double g_standby_y = 0.207;
double g_standby_yaw = 0.050;
int g_deprioritize_delivery_slot = 2;
int g_slot2_max_visits_before_accept = 2;
double g_coord_heartbeat_hz = 0.5;
std::string g_car_link_send_topic = "/car_link/send";
std::string g_car_link_recv_topic = "/car_link/recv";
std::string g_peer_connected_topic = "/car_link/peer_connected";
double g_peer_tcp_wait_timeout = 120.0;
bool g_require_peer_tcp = true;

std::vector<GoalTask> g_goal_list;
uint8_t g_station = move_nav::CarLink::STATION_UNKNOWN;
uint32_t g_carlink_seq = 0;
bool g_allow_start_round = false;
bool g_first_round = true;
bool g_pending_go_home = false;
std::atomic<bool> g_peer_tcp_connected(false);
std::mutex g_coord_mutex;
ros::Publisher g_car_link_pub;
MoveBaseClient* g_move_client_ptr = nullptr;

// 生成固定的识别板一模拟结果，方便视觉节点未完成时先调试导航流程。
Board1Result makeMockBoard1Result() {
    Board1Result result;
    result.has_a = true;
    result.has_b = true;
    result.has_c = true;
    result.delivery_slot = 1;
    result.sample_count = 3;
    return result;
}

// 生成固定的识别板二模拟结果，方便视觉节点未完成时先调试导航流程。
Board2Result makeMockBoard2Result() {
    Board2Result result;
    result.wait_seconds = 0;
    result.speech_text = "化验区空闲中，请快速通过";
    return result;
}

// 计算当前二维码包含了几个取样任务
int countBoard1Samples(const Board1Result& result) {
    return static_cast<int>(result.has_a) +
           static_cast<int>(result.has_b) +
           static_cast<int>(result.has_c);
}

// 安全检测二维码识别结果
bool normalizeBoard1Result(Board1Result* result) {
    if (result == nullptr) {
        return false;
    }

    const int sample_count = countBoard1Samples(*result);
    if (sample_count == 0) {
        ROS_WARN("二维码识别结果无 A/B/C 样本");
        return false;
    }

    if (result->delivery_slot < 1 || result->delivery_slot > 4) {
        ROS_ERROR("二维码识别返回的 delivery_slot 无效：%d", result->delivery_slot);
        return false;
    }

    if (result->sample_count != sample_count) {
        ROS_WARN("二维码识别 sample_count=%d 与 A/B/C 数量=%d 不一致，使用 A/B/C 数量",
                 result->sample_count, sample_count);
        result->sample_count = sample_count;
    }

    return true;
}

// 将音频文件路径进行播报。
// 使用小车原有方式播放 wav 文件：调用系统 aplay 命令。
void playAudioFile(const std::string& audio_file) {
    if (audio_file.empty()) {
        ROS_WARN("音频文件路径为空，跳过播放");
        return;
    }

    struct stat info;
    if (stat(audio_file.c_str(), &info) != 0 || !S_ISREG(info.st_mode)) {
        ROS_WARN("音频文件不存在，跳过播放：%s", audio_file.c_str());
        return;
    }

    ROS_INFO("播放音频文件：%s", audio_file.c_str());
    const std::string cmd = "aplay \"" + audio_file + "\"";
    const int ret = system(cmd.c_str());
    if (ret != 0) {
        ROS_WARN("音频播放命令执行失败：%s，返回值=%d", cmd.c_str(), ret);
    }
}

// 按约定生成完整音频文件路径：audio_dir/category/key.wav。
std::string audioPath(const std::string& category, const std::string& key) {
    const bool has_trailing_slash =
        !g_audio_dir.empty() &&
        (g_audio_dir[g_audio_dir.size() - 1] == '/' ||
         g_audio_dir[g_audio_dir.size() - 1] == '\\');
    return g_audio_dir + (has_trailing_slash ? "" : "/") + category + "/" + key + ".wav";
}

// 确保目录路径的末尾带有斜杠
std::string directoryWithTrailingSlash(const std::string& directory) {
    if (directory.empty()) {
        return directory;
    }

    const char last = directory[directory.size() - 1];
    return directory + ((last == '/' || last == '\\') ? "" : "/");
}

// 检查指定的路径在操作系统的文件系统中是否存在
bool directoryExists(const std::string& directory) {
    struct stat info;
    return stat(directory.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

// 如果目录不存在，创建整个路径
bool ensureDirectoryExists(const std::string& directory) {
    if (directory.empty()) {
        ROS_ERROR("截图保存目录为空");
        return false;
    }

    std::string target = directory;
    while (target.size() > 1 &&
           (target[target.size() - 1] == '/' || target[target.size() - 1] == '\\')) {
        target.erase(target.size() - 1);
    }

    if (directoryExists(target)) {
        return true;
    }

    std::string current;
    size_t pos = 0;
    if (!target.empty() && target[0] == '/') {
        current = "/";
        pos = 1;
    }

    while (pos <= target.size()) {
        const size_t next = target.find('/', pos);
        const std::string part =
            target.substr(pos, next == std::string::npos ? std::string::npos : next - pos);
        if (!part.empty()) {
            if (current.empty()) {
                current = part;
            } else if (current == "/") {
                current += part;
            } else {
                current += "/" + part;
            }

            if (!directoryExists(current) &&
                mkdir(current.c_str(), 0755) != 0 &&
                errno != EEXIST) {
                ROS_ERROR("创建截图保存目录失败：%s，错误：%s",
                          current.c_str(), strerror(errno));
                return false;
            }
        }

        if (next == std::string::npos) {
            break;
        }
        pos = next + 1;
    }

    return directoryExists(target);
}

// 生成最终图片保存的绝对路径
std::string snapshotImagePath(int image_index) {
    return directoryWithTrailingSlash(g_snapshot_dir) +
           std::to_string(image_index) + ".jpg";
}

// 将化验区目标编号转换为送样音频文件名中的窗口 key。
std::string slotKey(int delivery_slot) {
    static const char* keys[] = {"blood", "body_fluid", "immune", "hormone"};
    delivery_slot = std::max(1, std::min(4, delivery_slot));
    return keys[delivery_slot - 1];
}

// 将识别结果编号转换为取样播报音频文件名中的样本类型 key。
std::string sampleKey(int delivery_slot) {
    static const char* keys[] = {"venous_blood", "saliva", "tissue", "plasma"};
    delivery_slot = std::max(1, std::min(4, delivery_slot));
    return keys[delivery_slot - 1];
}

// 根据二维码/识别板一结果生成窗口组合 key，例如 A、AB、ABC。
std::string windowsKey(const Board1Result& result) {
    std::string key;
    if (result.has_a) {
        key += "A";
    }
    if (result.has_b) {
        key += "B";
    }
    if (result.has_c) {
        key += "C";
    }
    return key;
}

// 将导航点名称映射为裁判 task 字段（A/B/C/1–4/R）。
std::string goalNameToTask(const std::string& goal_name) {
    if (goal_name == "pickup_A") {
        return "A";
    }
    if (goal_name == "pickup_B") {
        return "B";
    }
    if (goal_name == "pickup_C") {
        return "C";
    }
    if (goal_name == "deliver_1") {
        return "1";
    }
    if (goal_name == "deliver_2") {
        return "2";
    }
    if (goal_name == "deliver_3") {
        return "3";
    }
    if (goal_name == "deliver_4") {
        return "4";
    }
    return g_default_task;
}

// 识别板二 → CV1，例如 WAIT-8。
std::string formatCV1(int wait_seconds) {
    return "WAIT-" + std::to_string(wait_seconds);
}

// 识别板一（二维码）→ CV2，例如 AB-1。
std::string formatCV2(const Board1Result& result) {
    return windowsKey(result) + "-" + std::to_string(result.delivery_slot);
}

void setCurrentTask(const std::string& task) {
    std::lock_guard<std::mutex> lock(g_judgement_mutex);
    g_current_task = task;
}

void updateBoard1Judgement(const Board1Result& result) {
    std::lock_guard<std::mutex> lock(g_judgement_mutex);
    g_cv2 = formatCV2(result);
}

void updateBoard2Judgement(const Board2Result& result) {
    std::lock_guard<std::mutex> lock(g_judgement_mutex);
    g_cv1 = formatCV1(result.wait_seconds);
}

void odomCB(const nav_msgs::OdometryConstPtr& msg) {
    std::lock_guard<std::mutex> lock(g_judgement_mutex);
    if (!g_use_tf_pose) {
        g_odom_x = msg->pose.pose.position.x;
        g_odom_y = msg->pose.pose.position.y;
    }
    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    g_speed = std::hypot(vx, vy);
}

void updatePoseFromTf() {
    if (!g_use_tf_pose || g_tf_buffer == nullptr) {
        return;
    }

    try {
        const geometry_msgs::TransformStamped tf = g_tf_buffer->lookupTransform(
            g_map_frame, g_base_frame, ros::Time(0), ros::Duration(0.05));
        std::lock_guard<std::mutex> lock(g_judgement_mutex);
        g_odom_x = tf.transform.translation.x;
        g_odom_y = tf.transform.translation.y;
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(5.0, "TF %s→%s 不可用，沿用上次坐标：%s",
                          g_map_frame.c_str(), g_base_frame.c_str(), ex.what());
    }
}

void judgementReportTimerCB(const ros::TimerEvent& /*event*/) {
    if (!g_enable_judgement_report) {
        return;
    }

    updatePoseFromTf();

    move_nav::JudgementReport report;
    {
        std::lock_guard<std::mutex> lock(g_judgement_mutex);
        report.id = g_car_id;
        report.speed = g_speed;
        report.odom = {g_odom_x, g_odom_y};
        report.task = g_current_task;
        report.CV1 = g_cv1.empty() ? g_default_cv1 : g_cv1;
        report.CV2 = g_cv2.empty() ? g_default_cv2 : g_cv2;
    }
    g_judgement_pub.publish(report);
}

// 将保存后的图片路径发给二维码识别服务，并接收 Board1Decode 结构化结果。
bool callBoard1Service(const std::string& image_path) {
    if (!g_board1_client.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("二维码识别服务不可用");
        return false;
    }

    move_nav::Board1Decode srv;
    srv.request.image_path = image_path;

    ROS_INFO("调用二维码识别服务：image_path=%s", image_path.c_str());
    if (!g_board1_client.call(srv)) {
        ROS_ERROR("调用二维码识别服务失败");
        return false;
    }

    g_board1_result.has_a = srv.response.has_a;
    g_board1_result.has_b = srv.response.has_b;
    g_board1_result.has_c = srv.response.has_c;
    g_board1_result.delivery_slot = srv.response.delivery_slot;
    g_board1_result.sample_count = srv.response.sample_count;

    if (!srv.response.error_message.empty()) {
        ROS_ERROR("二维码识别失败：%s", srv.response.error_message.c_str());
    }

    ROS_INFO("二维码识别服务返回：A=%d，B=%d，C=%d，delivery_slot=%d，sample_count=%d",
             g_board1_result.has_a,
             g_board1_result.has_b,
             g_board1_result.has_c,
             g_board1_result.delivery_slot,
             g_board1_result.sample_count);

    if (!srv.response.error_message.empty()) {
        return false;
    }

    return normalizeBoard1Result(&g_board1_result);
}

// 将保存后的图片路径发给文字识别服务，并接收识别板二的化验区状态结果。
bool callBoard2Service(const std::string& image_path) {
    if (!g_board2_client.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("识别板二文字识别服务不可用");
        return false;
    }

    move_nav::Board2Decode srv;
    srv.request.image_path = image_path;

    ROS_INFO("调用识别板二文字识别服务：image_path=%s", image_path.c_str());
    if (!g_board2_client.call(srv)) {
        ROS_ERROR("调用识别板二文字识别服务失败");
        return false;
    }

    g_board2_result.wait_seconds = srv.response.wait_seconds;
    g_board2_result.speech_text = srv.response.speech_text;
    ROS_INFO("识别板二文字识别返回：wait_seconds=%d，speech_text=%s",
             g_board2_result.wait_seconds,
             g_board2_result.speech_text.c_str());
    return true;
}

// 保存一帧相机图像。回调里不调用视觉服务，避免服务阻塞拖住 ROS 回调队列。
void snapshotCB(const sensor_msgs::ImageConstPtr& msg) {
    const VisionTask task = static_cast<VisionTask>(g_active_task.load());
    if (task == NoVisionTask) {
        return;
    }

    bool snapshot_ok = false;
    std::string image_path;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        image_path = snapshotImagePath(g_img_idx++);

        if (!cv::imwrite(image_path, cv_ptr->image)) {
            ROS_ERROR("保存图片失败：%s", image_path.c_str());
        } else {
            ROS_INFO("已保存图片：%s", image_path.c_str());
            snapshot_ok = true;
        }
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge 异常：%s", e.what());
    }

    {
        std::lock_guard<std::mutex> lock(g_snapshot_image_path_mutex);
        g_snapshot_image_path = image_path;
    }
    g_snapshot_ok.store(snapshot_ok);
    g_active_task.store(NoVisionTask);
    g_snapshot_done.store(true);
}

// 将药房业务点位转换成 move_base 可执行的导航目标。
move_base_msgs::MoveBaseGoal toMove(const GoalTask& goal_task) {
    ROS_INFO("正在前往 %s：(%.2f, %.2f, yaw=%.2f)",
             goal_task.name.c_str(), goal_task.x, goal_task.y, goal_task.yaw);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = goal_task.x;
    goal.target_pose.pose.position.y = goal_task.y;
    goal.target_pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, goal_task.yaw);
    goal.target_pose.pose.orientation.x = q.getX();
    goal.target_pose.pose.orientation.y = q.getY();
    goal.target_pose.pose.orientation.z = q.getZ();
    goal.target_pose.pose.orientation.w = q.getW();
    return goal;
}

// 发送 move_base 导航目标，并阻塞等待机器人到达或导航失败。
bool movetoPoint(const GoalTask& goal_task, MoveBaseClient& client) {
    const std::string task_for_goal = goalNameToTask(goal_task.name);
    if (task_for_goal == g_default_task) {
        setCurrentTask(g_default_task);
    }

    if (g_mock_navigation) {
        ROS_INFO("[模拟导航] 已到达 %s：(%.2f, %.2f, %.2f)",
                 goal_task.name.c_str(), goal_task.x, goal_task.y, goal_task.yaw);
        if (task_for_goal != g_default_task) {
            setCurrentTask(task_for_goal);
        }
        ++current_point;
        ros::Duration(0.1).sleep();
        return true;
    }

    ros::Rate rate(10);
    client.sendGoal(toMove(goal_task));

    // 目标刚发出去时通常会先进入 PENDING，再进入 ACTIVE。
    // 这里使用 WallTime，避免仿真时间 /clock 未发布或暂停时超时判断失效。
    const ros::WallTime start_deadline =
        ros::WallTime::now() + ros::WallDuration(g_navigation_start_timeout);
    while (ros::ok()) {
        const actionlib::SimpleClientGoalState state = client.getState();
        // 某些很近的目标可能还没观察到 ACTIVE 就已经 SUCCEEDED，直接进入后续成功处理。
        if (state == actionlib::SimpleClientGoalState::ACTIVE ||
            state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            break;
        }
        // 如果在启动阶段已经进入失败终态，立即返回，避免一直等 ACTIVE。
        if (state.isDone()) {
            ROS_ERROR("导航目标启动失败：%s，状态=%s",
                      goal_task.name.c_str(), state.toString().c_str());
            client.cancelGoal();
            return false;
        }
        if (ros::WallTime::now() >= start_deadline) {
            ROS_ERROR("导航目标启动超时：%s，等待 ACTIVE 超过 %.1f 秒，当前状态=%s",
                      goal_task.name.c_str(),
                      g_navigation_start_timeout,
                      state.toString().c_str());
            client.cancelGoal();
            return false;
        }
        ros::spinOnce();
        rate.sleep();
    }
    if (!ros::ok()) {
        client.cancelGoal();
        return false;
    }

    while (ros::ok()) {
        const actionlib::SimpleClientGoalState state = client.getState();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            break;
        }
        // 除 SUCCEEDED 外的所有终态都按导航失败处理，例如 ABORTED、REJECTED、PREEMPTED。
        if (state.isDone()) {
            ROS_ERROR("导航失败：%s，状态=%s",
                      goal_task.name.c_str(), state.toString().c_str());
            client.cancelGoal();
            return false;
        }
        ros::spinOnce();
        rate.sleep();
    }
    if (!ros::ok()) {
        client.cancelGoal();
        return false;
    }

    ROS_INFO("第 %zu 个点已到达：%s", current_point, goal_task.name.c_str());
    if (task_for_goal != g_default_task) {
        setCurrentTask(task_for_goal);
    }
    ++current_point;
    client.cancelGoal();
    ros::Duration(0.1).sleep();
    return true;
}

// 初始化航点表（含 rosparam 注入的 standby）。
void initGoalList() {
    g_goal_list.assign(GOAL_LIST.begin(), GOAL_LIST.end());
    const GoalTask standby = {g_standby_x, g_standby_y, g_standby_yaw, "standby"};
    if (g_goal_list.size() >= 1) {
        g_goal_list.insert(g_goal_list.begin() + 1, standby);
    } else {
        g_goal_list.push_back(standby);
    }
}

// 按任务点名称查找导航点，例如 board1_scan 或 pickup_A。
const GoalTask* findGoalByName(const std::string& name) {
    for (const GoalTask& goal : g_goal_list) {
        if (goal.name == name) {
            return &goal;
        }
    }
    return nullptr;
}

// slot 1/3/4 立即接受；deprioritize（默认 2）延后至 visit 达上限。
bool isPreferredDeliverySlot(int slot) {
    return slot == 1 || slot == 3 || slot == 4;
}

bool shouldAcceptDeliverySlot(int slot, int visit_round) {
    if (slot < 1 || slot > 4) {
        return false;
    }
    if (isPreferredDeliverySlot(slot)) {
        return true;
    }
    if (slot == g_deprioritize_delivery_slot) {
        if (visit_round >= g_slot2_max_visits_before_accept) {
            ROS_WARN("连续 %d 轮仍为 delivery_slot=%d，接受该任务",
                     visit_round, slot);
            return true;
        }
        return false;
    }
    return true;
}

void setStation(uint8_t station) {
    g_station = station;
}

void publishCarLink(uint8_t type, int delivery_slot = 0) {
    if (!g_dual_car_mode || !g_car_link_pub) {
        return;
    }
    move_nav::CarLink msg;
    msg.type = type;
    msg.from_id = g_car_id;
    msg.seq = ++g_carlink_seq;
    msg.stamp = ros::Time::now();
    msg.station = g_station;
    msg.delivery_slot = delivery_slot;
    g_car_link_pub.publish(msg);
    ROS_INFO("CarLink 发送 type=%u station=%u slot=%d seq=%u",
             type, g_station, delivery_slot, msg.seq);
}

void peerTcpConnectedCB(const std_msgs::Bool::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    const bool connected = msg->data;
    g_peer_tcp_connected.store(connected);
    if (!connected) {
        std::lock_guard<std::mutex> lock(g_coord_mutex);
        g_allow_start_round = false;
        g_pending_go_home = false;
        ROS_WARN("车际 TCP 已断开，暂停协调动作直至重连");
    } else {
        ROS_INFO("车际 TCP 已连接");
    }
}

void carLinkRecvCB(const move_nav::CarLink::ConstPtr& msg) {
    if (!g_dual_car_mode || !msg) {
        return;
    }
    if (msg->from_id == g_car_id) {
        return;
    }

    g_peer_tcp_connected.store(true);

    ROS_INFO("CarLink 收到 type=%u from=%s station=%u",
             msg->type, msg->from_id.c_str(), msg->station);

    if (msg->type == move_nav::CarLink::CARLINK_SCAN_OK) {
        std::lock_guard<std::mutex> lock(g_coord_mutex);
        if (g_station == move_nav::CarLink::STATION_STANDBY) {
            g_pending_go_home = true;
        }
    } else if (msg->type == move_nav::CarLink::CARLINK_ROUND_DONE) {
        std::lock_guard<std::mutex> lock(g_coord_mutex);
        g_allow_start_round = true;
        if (g_station == move_nav::CarLink::STATION_STANDBY) {
            g_pending_go_home = true;
            ROS_INFO("收到对端 ROUND_DONE，本车将从 standby 前往 home 后再开工");
        } else if (g_station == move_nav::CarLink::STATION_HOME) {
            ROS_INFO("收到对端 ROUND_DONE，本车（在 home）允许开始新一轮");
        }
    }
}

bool waitForPeerTcpLink() {
    if (!g_dual_car_mode || !g_require_peer_tcp) {
        return true;
    }

    ROS_INFO("等待车际 TCP 连接（话题 %s，超时 %.0fs）...",
             g_peer_connected_topic.c_str(), g_peer_tcp_wait_timeout);

    ros::Rate rate(5);
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(g_peer_tcp_wait_timeout);
    while (ros::ok() && ros::WallTime::now() < deadline) {
        if (g_peer_tcp_connected.load()) {
            ROS_INFO("车际 TCP 已就绪，允许导航");
            return true;
        }
        ros::spinOnce();
        rate.sleep();
    }

    ROS_ERROR("等待车际 TCP 连接超时（%.0fs）", g_peer_tcp_wait_timeout);
    return false;
}

void coordHeartbeatCB(const ros::TimerEvent& /*event*/) {
    publishCarLink(move_nav::CarLink::CARLINK_HEARTBEAT, 0);
}

// 双车：处理待命→home，并等待允许开工。
void processCoordinationIdle(MoveBaseClient& move_client) {
    if (!g_dual_car_mode) {
        return;
    }

    ros::Rate rate(10);
    while (ros::ok()) {
        if (!g_peer_tcp_connected.load()) {
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        bool can_start = false;
        {
            std::lock_guard<std::mutex> lock(g_coord_mutex);
            const bool at_home = (g_station == move_nav::CarLink::STATION_HOME);
            const bool is_first_mover = (g_car_id == g_first_mover_car_id);
            const bool first_round_start =
                g_first_round && is_first_mover && g_allow_start_round == false;
            // 2 号车：必须收到 1 号 ROUND_DONE 且已在 home；1 号车首轮可在 home 直接开工
            can_start = at_home &&
                        (g_allow_start_round || first_round_start);
        }
        if (can_start) {
            if (g_first_round) {
                g_first_round = false;
            }
            {
                std::lock_guard<std::mutex> lock(g_coord_mutex);
                g_allow_start_round = false;
            }
            return;
        }

        {
            std::lock_guard<std::mutex> lock(g_coord_mutex);
            if (g_pending_go_home &&
                g_station == move_nav::CarLink::STATION_STANDBY &&
                g_peer_tcp_connected.load()) {
                const GoalTask* home_goal = findGoalByName("home");
                if (home_goal != nullptr) {
                    ROS_INFO("收到对端 SCAN_OK，预备点 → home");
                    if (movetoPoint(*home_goal, move_client)) {
                        setStation(move_nav::CarLink::STATION_HOME);
                        publishCarLink(move_nav::CarLink::CARLINK_GO_HOME_ACK, 0);
                    }
                    g_pending_go_home = false;
                }
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
}

bool navigateToInitialStation(MoveBaseClient& move_client) {
    const std::string target =
        (g_initial_station == "standby") ? "standby" : "home";
    const GoalTask* goal = findGoalByName(target);
    if (goal == nullptr) {
        ROS_ERROR("找不到初始站位：%s", target.c_str());
        return false;
    }
    ROS_INFO("双车模式：前往初始站位 %s", target.c_str());
    if (!movetoPoint(*goal, move_client)) {
        return false;
    }
    if (target == "standby") {
        setStation(move_nav::CarLink::STATION_STANDBY);
    } else {
        setStation(move_nav::CarLink::STATION_HOME);
    }
    return true;
}

// 请求识别板一截图；回调只保存图片，本函数拿到图片路径后再调用二维码识别服务。
bool requestBoard1Vision(double timeout_sec, Board1Result* result) {
    if (g_use_mock_data) {
        if (result != nullptr) {
            *result = makeMockBoard1Result();
            normalizeBoard1Result(result);
        }
        ROS_INFO("[模拟数据] 使用识别板一假结果");
        return true;
    }

    g_service_ok = false;
    g_snapshot_done.store(false);
    g_snapshot_ok.store(false);
    {
        std::lock_guard<std::mutex> lock(g_snapshot_image_path_mutex);
        g_snapshot_image_path.clear();
    }
    g_active_task.store(Board1Decode);

    ros::Rate rate(20);
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(timeout_sec);
    while (ros::ok() && ros::WallTime::now() < deadline) {
        ros::spinOnce();
        if (g_snapshot_done.load()) {
            if (!g_snapshot_ok.load()) {
                g_service_ok = false;
                return false;
            }

            ROS_INFO("调用视觉任务：board1_decode");
            std::string image_path;
            {
                std::lock_guard<std::mutex> lock(g_snapshot_image_path_mutex);
                image_path = g_snapshot_image_path;
            }
            g_service_ok = callBoard1Service(image_path);
            if (g_service_ok && result != nullptr) {
                *result = g_board1_result;
            }
            return g_service_ok;
        }
        rate.sleep();
    }

    g_active_task.store(NoVisionTask);
    ROS_WARN("识别板一视觉服务等待超时");
    return false;
}

// 请求识别板二截图；回调只保存图片，本函数拿到图片路径后再调用文字识别服务。
bool requestBoard2Vision(double timeout_sec, Board2Result* result) {
    if (g_use_mock_data) {
        if (result != nullptr) {
            *result = makeMockBoard2Result();
        }
        ROS_INFO("[模拟数据] 使用识别板二假结果");
        return true;
    }

    g_service_ok = false;
    g_snapshot_done.store(false);
    g_snapshot_ok.store(false);
    {
        std::lock_guard<std::mutex> lock(g_snapshot_image_path_mutex);
        g_snapshot_image_path.clear();
    }
    g_active_task.store(Board2Decode);

    ros::Rate rate(20);
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(timeout_sec);
    while (ros::ok() && ros::WallTime::now() < deadline) {
        ros::spinOnce();
        if (g_snapshot_done.load()) {
            if (!g_snapshot_ok.load()) {
                g_service_ok = false;
                return false;
            }

            ROS_INFO("调用视觉任务：board2_decode");
            std::string image_path;
            {
                std::lock_guard<std::mutex> lock(g_snapshot_image_path_mutex);
                image_path = g_snapshot_image_path;
            }
            g_service_ok = callBoard2Service(image_path);
            if (g_service_ok && result != nullptr) {
                *result = g_board2_result;
            }
            return g_service_ok;
        }
        rate.sleep();
    }

    g_active_task.store(NoVisionTask);
    ROS_WARN("识别板二视觉服务等待超时");
    return false;
}

// 识别板一：同一点位最多尝试 3 次；连续失败后回 home，再前往 board1_scan 重试，直至成功。
bool scanBoard1WithRetry(MoveBaseClient& move_client, Board1Result* result) {
    const GoalTask* board1_goal = findGoalByName("board1_scan");
    const GoalTask* home_goal = findGoalByName("home");
    if (board1_goal == nullptr) {
        ROS_ERROR("GOAL_LIST 中没有 board1_scan 点位");
        return false;
    }

    constexpr int kMaxAttemptsPerVisit = 3;
    int visit_round = 0;

    while (ros::ok()) {
        ++visit_round;
        ROS_INFO("前往识别板一（第 %d 轮）", visit_round);
        if (!movetoPoint(*board1_goal, move_client)) {
            return false;
        }

        for (int attempt = 1; attempt <= kMaxAttemptsPerVisit; ++attempt) {
            ROS_INFO("识别板一二维码识别，第 %d/%d 次尝试", attempt, kMaxAttemptsPerVisit);
            Board1Result candidate;
            if (requestBoard1Vision(15.0, &candidate)) {
                if (!shouldAcceptDeliverySlot(candidate.delivery_slot, visit_round)) {
                    ROS_WARN(
                        "delivery_slot=%d 优先级低于 1/3/4（visit=%d/%d），继续重扫",
                        candidate.delivery_slot,
                        visit_round,
                        g_slot2_max_visits_before_accept);
                    if (attempt < kMaxAttemptsPerVisit) {
                        ros::Duration(1.0).sleep();
                    }
                    continue;
                }
                if (result != nullptr) {
                    *result = candidate;
                }
                publishCarLink(move_nav::CarLink::CARLINK_SCAN_OK,
                               candidate.delivery_slot);
                return true;
            }
            ROS_WARN("识别板一二维码识别失败（第 %d/%d 次）", attempt, kMaxAttemptsPerVisit);
            if (attempt < kMaxAttemptsPerVisit) {
                ros::Duration(1.0).sleep();
            }
        }

        ROS_WARN("识别板一连续 %d 次失败，返回 home 后重新前往识别", kMaxAttemptsPerVisit);
        if (home_goal != nullptr) {
            if (!movetoPoint(*home_goal, move_client)) {
                return false;
            }
        } else {
            ROS_WARN("GOAL_LIST 中没有 home 点位，将在 board1_scan 直接开始下一轮识别");
        }
    }

    return false;
}

// 执行一轮完整药房任务：识别板一、取样、识别板二、送样。
bool runOneQrMission(MoveBaseClient& move_client) {
    ROS_INFO("========== 开始一轮药房任务 ==========");

    Board1Result board1_result;
    if (!scanBoard1WithRetry(move_client, &board1_result)) {
        ROS_ERROR("识别板一流程中止");
        return false;
    }
    updateBoard1Judgement(board1_result);

    ROS_INFO("识别板一结果：A=%d，B=%d，C=%d，delivery_slot=%d，sample_count=%d",
             board1_result.has_a,
             board1_result.has_b,
             board1_result.has_c,
             board1_result.delivery_slot,
             board1_result.sample_count);

    std::vector<std::string> pickup_route;
    if (board1_result.has_c) {
        pickup_route.push_back("pickup_C");
    }
    if (board1_result.has_a) {
        pickup_route.push_back("pickup_A");
    }
    if (board1_result.has_b) {
        pickup_route.push_back("pickup_B");
    }

    for (const std::string& goal_name : pickup_route) {
        const GoalTask* goal = findGoalByName(goal_name);
        if (goal == nullptr) {
            ROS_ERROR("GOAL_LIST 中没有取样点位：%s", goal_name.c_str());
            return false;
        }
        if (!movetoPoint(*goal, move_client)) {
            return false;
        }

        ros::Duration(1.5).sleep();
        const std::string window_name = goal_name.substr(goal_name.size() - 1);
        ROS_INFO("已取到样本：source_slot=%s", window_name.c_str());
    }
    
    // 根据识别结果生成取样播报音频文件名，并播放。
    const std::string pickup_key =
        windowsKey(board1_result) + "_" + sampleKey(board1_result.delivery_slot);
    playAudioFile(audioPath("pickup", pickup_key));

    const GoalTask* board2_goal = findGoalByName("board2_scan");
    if (board2_goal == nullptr) {
        ROS_ERROR("GOAL_LIST 中没有 board2_scan 点位");
        return false;
    }
    if (!movetoPoint(*board2_goal, move_client)) {
        return false;
    }

    Board2Result board2_result;
    if (!requestBoard2Vision(15.0, &board2_result)) {
        ROS_WARN("识别板二视觉任务失败或超时，默认化验区空闲");
        board2_result.wait_seconds = 0;
        board2_result.speech_text = "化验区空闲中，请快速通过";
    }
    updateBoard2Judgement(board2_result);

    const std::string board2_key =
        board2_result.wait_seconds > 0 ? "wait_" + std::to_string(board2_result.wait_seconds)
                                       : "free";
    if (!board2_result.speech_text.empty()) {
        ROS_INFO("识别板二服务返回文本：%s", board2_result.speech_text.c_str());
    }
    playAudioFile(audioPath("board2", board2_key));
    if (board2_result.wait_seconds > 0) {
        ROS_INFO("化验区忙碌，等待 %d 秒后再通过", board2_result.wait_seconds);
        ros::Duration(board2_result.wait_seconds).sleep();
    }

    const std::string delivery_goal_name =
        "deliver_" + std::to_string(board1_result.delivery_slot);
    const GoalTask* delivery_goal = findGoalByName(delivery_goal_name);
    if (delivery_goal == nullptr) {
        ROS_ERROR("GOAL_LIST 中没有送样点位：%s", delivery_goal_name.c_str());
        return false;
    }
    if (!movetoPoint(*delivery_goal, move_client)) {
        return false;
    }

    ros::Duration(1.5).sleep();
    ROS_INFO("样本已送达：delivery_slot=%d，count=%d",
             board1_result.delivery_slot, board1_result.sample_count);
    playAudioFile(audioPath("delivery",
                            slotKey(board1_result.delivery_slot) + "_" +
                                std::to_string(board1_result.sample_count)));
    ROS_INFO("========== 一轮药房任务完成 ==========");
    return true;
}

// 初始化 ROS 通信接口，并循环执行药房配送任务。
int main(int argc, char* argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "yaofang_control_service_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string board1_service = "/yaofang_vision/board1_decode";
    std::string board2_service = "/yaofang_vision/board2_decode";
    pnh.param("use_mock_data", g_use_mock_data, g_use_mock_data);
    pnh.param("mock_navigation", g_mock_navigation, g_mock_navigation);
    pnh.param("max_rounds", g_max_rounds, g_max_rounds);
    pnh.param("vision_service_wait_timeout", g_vision_service_wait_timeout, g_vision_service_wait_timeout);
    pnh.param("move_base_wait_timeout", g_move_base_wait_timeout, g_move_base_wait_timeout);
    pnh.param("navigation_start_timeout", g_navigation_start_timeout, g_navigation_start_timeout);
    pnh.param("board1_detection_service", board1_service, board1_service);
    pnh.param("board2_detection_service", board2_service, board2_service);
    pnh.param("audio_dir", g_audio_dir, g_audio_dir);
    pnh.param("snapshot_dir", g_snapshot_dir, g_snapshot_dir);
    std::string image_topic = "/camera/rgb/image_raw";
    pnh.param("image_topic", image_topic, image_topic);
    pnh.param("car_id", g_car_id, g_car_id);
    pnh.param("enable_judgement_report", g_enable_judgement_report, g_enable_judgement_report);
    pnh.param("judgement_report_rate", g_judgement_report_rate, g_judgement_report_rate);
    pnh.param("judgement_report_topic", g_judgement_report_topic, g_judgement_report_topic);
    pnh.param("default_cv1", g_default_cv1, g_default_cv1);
    pnh.param("default_cv2", g_default_cv2, g_default_cv2);
    pnh.param("default_task", g_default_task, g_default_task);
    g_current_task = g_default_task;
    g_cv1 = g_default_cv1;

    std::string odom_topic = "/odom";
    pnh.param("odom_topic", odom_topic, odom_topic);
    pnh.param("use_tf_pose", g_use_tf_pose, g_use_tf_pose);
    pnh.param("map_frame", g_map_frame, g_map_frame);
    pnh.param("base_frame", g_base_frame, g_base_frame);
    pnh.param("dual_car_mode", g_dual_car_mode, g_dual_car_mode);
    pnh.param("initial_station", g_initial_station, g_initial_station);
    pnh.param("first_mover_car_id", g_first_mover_car_id, g_first_mover_car_id);
    pnh.param("standby_x", g_standby_x, g_standby_x);
    pnh.param("standby_y", g_standby_y, g_standby_y);
    pnh.param("standby_yaw", g_standby_yaw, g_standby_yaw);
    pnh.param("deprioritize_delivery_slot", g_deprioritize_delivery_slot,
              g_deprioritize_delivery_slot);
    pnh.param("slot2_max_visits_before_accept", g_slot2_max_visits_before_accept,
              g_slot2_max_visits_before_accept);
    pnh.param("coord_heartbeat_hz", g_coord_heartbeat_hz, g_coord_heartbeat_hz);
    pnh.param("car_link_send_topic", g_car_link_send_topic, g_car_link_send_topic);
    pnh.param("car_link_recv_topic", g_car_link_recv_topic, g_car_link_recv_topic);
    pnh.param("peer_connected_topic", g_peer_connected_topic, g_peer_connected_topic);
    pnh.param("peer_tcp_wait_timeout", g_peer_tcp_wait_timeout, g_peer_tcp_wait_timeout);
    pnh.param("require_peer_tcp", g_require_peer_tcp, g_require_peer_tcp);

    initGoalList();

    if (!ensureDirectoryExists(g_snapshot_dir)) {
        ROS_ERROR("截图保存目录不可用，主程序停止：%s", g_snapshot_dir.c_str());
        return 1;
    }

    MoveBaseClient move_client("move_base", true);
    g_move_client_ptr = &move_client;

    ros::Subscriber car_link_sub;
    ros::Subscriber peer_connected_sub;
    ros::Timer coord_timer;
    if (g_dual_car_mode) {
        g_car_link_pub = nh.advertise<move_nav::CarLink>(g_car_link_send_topic, 10);
        car_link_sub = nh.subscribe(g_car_link_recv_topic, 10, carLinkRecvCB);
        peer_connected_sub =
            nh.subscribe(g_peer_connected_topic, 1, peerTcpConnectedCB);
        const double hb_hz = std::max(0.1, g_coord_heartbeat_hz);
        coord_timer = nh.createTimer(ros::Duration(1.0 / hb_hz), coordHeartbeatCB);
    }

    ros::Subscriber image_sub = nh.subscribe(image_topic, 1, snapshotCB);
    ros::Subscriber odom_sub = nh.subscribe(odom_topic, 10, odomCB);
    if (g_use_tf_pose) {
        g_tf_buffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer());
        g_tf_listener = std::unique_ptr<tf2_ros::TransformListener>(
            new tf2_ros::TransformListener(*g_tf_buffer));
    }
    g_board1_client = nh.serviceClient<move_nav::Board1Decode>(board1_service);
    g_board2_client = nh.serviceClient<move_nav::Board2Decode>(board2_service);

    ros::Timer judgement_timer;
    if (g_enable_judgement_report) {
        g_judgement_pub =
            nh.advertise<move_nav::JudgementReport>(g_judgement_report_topic, 10);
        const double report_rate = std::max(0.1, g_judgement_report_rate);
        judgement_timer = nh.createTimer(
            ros::Duration(1.0 / report_rate), judgementReportTimerCB);
    }

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ROS_INFO("=== 直接服务调用版药房控制节点已启动 ===");
    ROS_INFO("参数：use_mock_data=%d，mock_navigation=%d，max_rounds=%d，vision_service_wait_timeout=%.1f，move_base_wait_timeout=%.1f，navigation_start_timeout=%.1f",
             g_use_mock_data,
             g_mock_navigation,
             g_max_rounds,
             g_vision_service_wait_timeout,
             g_move_base_wait_timeout,
             g_navigation_start_timeout);
    ROS_INFO("视觉服务：board1=%s，board2=%s",
             board1_service.c_str(), board2_service.c_str());
    ROS_INFO("语音目录：%s", directoryWithTrailingSlash(g_audio_dir).c_str());
    ROS_INFO("截图保存目录：%s", directoryWithTrailingSlash(g_snapshot_dir).c_str());
    ROS_INFO("图像订阅话题：%s", image_topic.c_str());
    ROS_INFO("裁判上报：enable=%d，car_id=%s，topic=%s，rate=%.2f Hz，odom=%s，use_tf_pose=%d",
             g_enable_judgement_report,
             g_car_id.c_str(),
             g_judgement_report_topic.c_str(),
             g_judgement_report_rate,
             odom_topic.c_str(),
             g_use_tf_pose);
    ROS_INFO("双车：dual_car_mode=%d，initial_station=%s，first_mover=%s，standby=(%.2f,%.2f,%.2f)",
             g_dual_car_mode,
             g_initial_station.c_str(),
             g_first_mover_car_id.c_str(),
             g_standby_x,
             g_standby_y,
             g_standby_yaw);
    ROS_INFO("板一 slot 优先级：deprioritize=%d，slot2_accept_after_visits=%d",
             g_deprioritize_delivery_slot,
             g_slot2_max_visits_before_accept);

    if (!g_use_mock_data) {
        ROS_INFO("等待二维码识别服务：%s", board1_service.c_str());
        if (!g_board1_client.waitForExistence(ros::Duration(g_vision_service_wait_timeout))) {
            ROS_ERROR("二维码识别服务未就绪，主程序停止：%s", board1_service.c_str());
            return 1;
        }
        ROS_INFO("二维码识别服务已连接");

        ROS_INFO("等待识别板二文字识别服务：%s", board2_service.c_str());
        if (!g_board2_client.waitForExistence(ros::Duration(g_vision_service_wait_timeout))) {
            ROS_ERROR("识别板二文字识别服务未就绪，主程序停止：%s", board2_service.c_str());
            return 1;
        }
        ROS_INFO("识别板二文字识别服务已连接");
    }

    if (!g_mock_navigation) {
        ROS_INFO("等待 move_base action server...");
        if (!move_client.waitForServer(ros::Duration(g_move_base_wait_timeout))) {
            ROS_ERROR("move_base action server 未就绪，主程序停止，等待超时 %.1f 秒",
                      g_move_base_wait_timeout);
            return 1;
        }
        ROS_INFO("已连接 move_base action server");
    } else {
        ROS_INFO("[模拟导航] 跳过 move_base action server");
    }

    if (g_dual_car_mode) {
        if (!waitForPeerTcpLink()) {
            ROS_ERROR("双车模式：车际 TCP 未连接，主程序停止");
            return 1;
        }
        if (!navigateToInitialStation(move_client)) {
            ROS_ERROR("无法到达初始站位");
            return 1;
        }
    }

    int completed_rounds = 0;
    while (ros::ok() && (g_max_rounds <= 0 || completed_rounds < g_max_rounds)) {
        current_point = 0;

        if (g_dual_car_mode) {
            processCoordinationIdle(move_client);
            setStation(move_nav::CarLink::STATION_ON_MISSION);
        }

        const bool ok = runOneQrMission(move_client);

        if (!ok) {
            return 1;
        }

        if (g_dual_car_mode) {
            const GoalTask* standby_goal = findGoalByName("standby");
            if (standby_goal != nullptr) {
                if (!movetoPoint(*standby_goal, move_client)) {
                    return 1;
                }
            }
            setStation(move_nav::CarLink::STATION_STANDBY);
            publishCarLink(move_nav::CarLink::CARLINK_ROUND_DONE, 0);
        } else {
            const GoalTask* home_goal = findGoalByName("home");
            if (home_goal != nullptr) {
                movetoPoint(*home_goal, move_client);
            }
        }

        ++completed_rounds;
        ROS_INFO("第 %d 轮任务完成", completed_rounds);
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("控制节点停止，已完成 %d 轮任务", completed_rounds);
    return 0;
}
