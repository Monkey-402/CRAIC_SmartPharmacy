#include <algorithm>
#include <atomic>
#include <clocale>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <cv_bridge/cv_bridge.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>

#include "move_nav/Board1Decode.h"
#include "move_nav/Board2Decode.h"

#ifndef SAVE_DIR
#define SAVE_DIR "/home/zinn/snapshots/"
#endif

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

// 任务点
const std::vector<GoalTask> GOAL_LIST = {
    {1.463, 3.840, -3.14, "home"},
    {0.680, 3.840, -3.14, "board1_scan"},
    {0.865, 1.022, -1.57, "pickup_A"},
    {0.001, 0.499, -1.57, "pickup_B"},
    {0.001, 1.526, -1.57, "pickup_C"},
    {2.284, -0.260, 0, "board2_scan"},
    {3.415, 1.435, 1.57, "deliver_1"},
    {2.540, 2.035, 1.57, "deliver_2"},
    {3.415, 2.515, 1.57, "deliver_3"},
    {2.602, 3.030, 1.57, "deliver_4"}
};

ros::Publisher g_audio_play_pub;
ros::ServiceClient g_board1_client;
ros::ServiceClient g_board2_client;

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
double g_navigation_start_timeout = 10.0;
size_t current_point = 0;

bool g_service_done = false;
bool g_service_ok = false;
Board1Result g_board1_result;
Board2Result g_board2_result;

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

int countBoard1Samples(const Board1Result& result) {
    return static_cast<int>(result.has_a) +
           static_cast<int>(result.has_b) +
           static_cast<int>(result.has_c);
}

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

// 将音频文件路径发布到语音播放话题。
void playAudioFile(const std::string& audio_file) {
    if (audio_file.empty()) {
        ROS_WARN("音频文件路径为空，跳过播放");
        return;
    }

    std_msgs::String msg;
    msg.data = audio_file;
    g_audio_play_pub.publish(msg);
    ROS_INFO("播放音频文件：%s", audio_file.c_str());
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

    ROS_INFO("二维码识别服务返回：A=%d，B=%d，C=%d，delivery_slot=%d，sample_count=%d",
             g_board1_result.has_a,
             g_board1_result.has_b,
             g_board1_result.has_c,
             g_board1_result.delivery_slot,
             g_board1_result.sample_count);

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

// 保存一帧相机图像，并执行当前等待中的识别板一或识别板二服务请求。
void snapshotCB(const sensor_msgs::ImageConstPtr& msg) {
    const VisionTask task = static_cast<VisionTask>(g_active_task.load());
    if (task == NoVisionTask) {
        return;
    }
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        const std::string image_path =
            std::string(SAVE_DIR) + std::to_string(g_img_idx++) + ".jpg";

        if (!cv::imwrite(image_path, cv_ptr->image)) {
            ROS_ERROR("保存图片失败：%s", image_path.c_str());
            g_service_ok = false;
        } else {
            ROS_INFO("已保存图片：%s", image_path.c_str());
            switch (task) {
                case Board1Decode:
                    ROS_INFO("调用视觉任务：board1_decode");
                    g_service_ok = callBoard1Service(image_path);
                    break;
                case Board2Decode:
                    ROS_INFO("调用视觉任务：board2_decode");
                    g_service_ok = callBoard2Service(image_path);
                    break;
                default:
                    ROS_ERROR("未知的视觉任务类型：%d", static_cast<int>(task));
                    g_service_ok = false;
                    break;
                }
        }
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge 异常：%s", e.what());
        g_service_ok = false;
    }

    g_active_task.store(NoVisionTask);
    g_service_done = true;
}

// 将药房业务点位转换成 move_base 可执行的导航目标。
move_base_msgs::MoveBaseGoal toMove(const GoalTask& goal_task) {
    ROS_INFO("正在前往 %s：(%.2f, %.2f, %.2f)",
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
    if (g_mock_navigation) {
        ROS_INFO("[模拟导航] 已到达 %s：(%.2f, %.2f, %.2f)",
                 goal_task.name.c_str(), goal_task.x, goal_task.y, goal_task.yaw);
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
    ++current_point;
    client.cancelGoal();
    ros::Duration(0.1).sleep();
    return true;
}

// 按任务点名称查找导航点，例如 board1_scan 或 pickup_A。
const GoalTask* findGoalByName(const std::string& name) {
    for (const GoalTask& goal : GOAL_LIST) {
        if (goal.name == name) {
            return &goal;
        }
    }
    return nullptr;
}

// 请求识别板一截图，并在 snapshotCB 中调用二维码识别服务返回结果。
bool requestBoard1Vision(double timeout_sec, Board1Result* result) {
    if (g_use_mock_data) {
        if (result != nullptr) {
            *result = makeMockBoard1Result();
            normalizeBoard1Result(result);
        }
        ROS_INFO("[模拟数据] 使用识别板一假结果");
        return true;
    }

    g_service_done = false;
    g_service_ok = false;
    g_active_task.store(Board1Decode);

    ros::Rate rate(20);
    const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_sec);
    while (ros::ok() && ros::Time::now() < deadline) {
        ros::spinOnce();
        if (g_service_done) {
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

// 请求识别板二截图，并在 snapshotCB 中调用文字识别服务返回结果。
bool requestBoard2Vision(double timeout_sec, Board2Result* result) {
    if (g_use_mock_data) {
        if (result != nullptr) {
            *result = makeMockBoard2Result();
        }
        ROS_INFO("[模拟数据] 使用识别板二假结果");
        return true;
    }

    g_service_done = false;
    g_service_ok = false;
    g_active_task.store(Board2Decode);

    ros::Rate rate(20);
    const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_sec);
    while (ros::ok() && ros::Time::now() < deadline) {
        ros::spinOnce();
        if (g_service_done) {
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

// 执行一轮完整药房任务：识别板一、取样、识别板二、送样。
bool runOneQrMission(MoveBaseClient& move_client) {
    ROS_INFO("========== 开始一轮药房任务 ==========");

    const GoalTask* board1_goal = findGoalByName("board1_scan");
    if (board1_goal == nullptr) {
        ROS_ERROR("GOAL_LIST 中没有 board1_scan 点位");
        return false;
    }
    if (!movetoPoint(*board1_goal, move_client)) {
        return false;
    }

    Board1Result board1_result;
    if (!requestBoard1Vision(15.0, &board1_result)) {
        ROS_ERROR("识别板一二维码识别失败");
        return false;
    }

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

    std::string pickup_windows;
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
        if (!pickup_windows.empty()) {
            pickup_windows += ",";
        }
        pickup_windows += window_name;
        ROS_INFO("已取到样本：source_slot=%s", window_name.c_str());
    }
    
    
    const char* sample_type = "静脉血样本";
    if (board1_result.delivery_slot == 2) {
        sample_type = "唾液样本";
    } else if (board1_result.delivery_slot == 3) {
        sample_type = "组织样本";
    } else if (board1_result.delivery_slot == 4) {
        sample_type = "血浆样本";
    }

    ROS_INFO("取样播报：取到 %s 窗口中的 %s",
             pickup_windows.c_str(), sample_type);
    playAudioFile("/path/to/pickup_summary.mp3");

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

    board2_result.wait_seconds = std::max(0, board2_result.wait_seconds);
    //如果识别消息为空
    if (board2_result.speech_text.empty()) {
        board2_result.speech_text =
            board2_result.wait_seconds > 0 ? "化验区忙碌中，请等待"
                                           : "化验区空闲中，请快速通过";
    }

    ROS_INFO("识别板二播报：%s", board2_result.speech_text.c_str());
    playAudioFile("/path/to/board2_notice.mp3");
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

    const char* delivery_window = "血常规窗口";
    if (board1_result.delivery_slot == 2) {
        delivery_window = "体液窗口";
    } else if (board1_result.delivery_slot == 3) {
        delivery_window = "免疫检测窗口";
    } else if (board1_result.delivery_slot == 4) {
        delivery_window = "激素检验窗口";
    }

    ros::Duration(1.5).sleep();
    ROS_INFO("样本已送达：delivery_slot=%d，count=%d",
             board1_result.delivery_slot, board1_result.sample_count);
    ROS_INFO("送样播报：到达 %s，样本数为%d",
             delivery_window, board1_result.sample_count);
    playAudioFile("/path/to/delivery_notice.mp3");

    playAudioFile("/path/to/mission_done.mp3");
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

    MoveBaseClient move_client("move_base", true);
    ros::Subscriber image_sub = nh.subscribe("/camera/image_raw", 1, snapshotCB);
    g_audio_play_pub = nh.advertise<std_msgs::String>("/smartcommunity/audio_play", 10);
    g_board1_client = nh.serviceClient<move_nav::Board1Decode>(board1_service);
    g_board2_client = nh.serviceClient<move_nav::Board2Decode>(board2_service);

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

    int completed_rounds = 0;
    while (ros::ok() && (g_max_rounds <= 0 || completed_rounds < g_max_rounds)) {
        current_point = 0;
        const bool ok = runOneQrMission(move_client);
        const GoalTask* home_goal = findGoalByName("home");
        if (home_goal != nullptr) {
            movetoPoint(*home_goal, move_client);
        }

        if (!ok) {
            playAudioFile("/path/to/task_error.mp3");
            return 1;
        }

        ++completed_rounds;
        ROS_INFO("第 %d 轮任务完成", completed_rounds);
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("控制节点停止，已完成 %d 轮任务", completed_rounds);
    return 0;
}
