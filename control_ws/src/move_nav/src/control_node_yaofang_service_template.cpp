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
*   int32 wait_seconds #等待秒数
*   string speech_text #识别文字
发送：
*   string image_path  #图片路径
*/

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
    int delivery_slot = 1;
    int sample_count = 0;
};

struct Board2Result {
    int wait_seconds = 0;
    std::string speech_text;
};

const std::vector<GoalTask> GOAL_LIST = {
    {1.210, 3.725, -3.062, "home"},
    {0.683, 3.692, -3.104, "board1_scan"},
    {0.820, 0.973, -1.57, "pickup_A"},
    {0.001, 0.499, -1.678, "pickup_B"},
    {0.043, 1.501, -1.692, "pickup_C"},
    {2.265, 0.202, -0.048, "board2_scan"},
    {3.407, 1.470, 1.670, "deliver_1"},
    {2.675, 2.089, 1.260, "deliver_2"},
    {3.382, 2.515, 1.666, "deliver_3"},
    {2.602, 3.030, 1.543, "deliver_4"}
};

ros::ServiceClient g_board1_client;
ros::ServiceClient g_board2_client;
std::string g_audio_dir = "/home/EPRobot/audio/yaofang/";

static std::atomic<int> g_img_idx(0);
static std::atomic<int> g_active_task(NoVisionTask);

bool g_use_mock_data = false;
bool g_mock_navigation = false;
int g_max_rounds = 0;
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

// 使用小车原有方式播放 wav 文件：调用系统 aplay 命令。
void playAudioFile(const std::string& audio_file) {
    if (audio_file.empty()) {
        ROS_WARN("音频文件路径为空，跳过播放");
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

// 将保存后的图片路径发给识别板一服务，并直接接收结构化识别结果。
bool callBoard1Service(const std::string& image_path) {
    if (!g_board1_client.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("识别板一视觉服务不可用");
        return false;
    }

    move_nav::Board1Decode srv;
    srv.request.image_path = image_path;

    ROS_INFO("调用识别板一视觉服务：image_path=%s", image_path.c_str());
    if (!g_board1_client.call(srv)) {
        ROS_ERROR("调用识别板一视觉服务失败");
        return false;
    }

    g_board1_result.has_a = srv.response.has_a;
    g_board1_result.has_b = srv.response.has_b;
    g_board1_result.has_c = srv.response.has_c;
    g_board1_result.delivery_slot = srv.response.delivery_slot;
    g_board1_result.sample_count = srv.response.sample_count;

    return true;
}

// 将保存后的图片路径发给识别板二服务，并直接接收化验区状态结果。
bool callBoard2Service(const std::string& image_path) {
    if (!g_board2_client.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("识别板二视觉服务不可用");
        return false;
    }

    move_nav::Board2Decode srv;
    srv.request.image_path = image_path;

    ROS_INFO("调用识别板二视觉服务：image_path=%s", image_path.c_str());
    if (!g_board2_client.call(srv)) {
        ROS_ERROR("调用识别板二视觉服务失败");
        return false;
    }

    g_board2_result.wait_seconds = srv.response.wait_seconds;
    g_board2_result.speech_text = srv.response.speech_text;
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

    while (ros::ok() && client.getState() != actionlib::SimpleClientGoalState::ACTIVE) {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok() && client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        const actionlib::SimpleClientGoalState state = client.getState();
        if (state == actionlib::SimpleClientGoalState::ABORTED ||
            state == actionlib::SimpleClientGoalState::REJECTED ||
            state == actionlib::SimpleClientGoalState::LOST) {
            ROS_ERROR("导航失败：%s，状态=%s",
                      goal_task.name.c_str(), state.toString().c_str());
            client.cancelGoal();
            return false;
        }
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("第 %zu 个点已到达：%s", current_point, goal_task.name.c_str());
    ++current_point;
    client.cancelGoal();
    ros::Duration(0.1).sleep();
    return true;
}

// 按业务点名称查找导航点，例如 board1_scan 或 pickup_A。
const GoalTask* findGoalByName(const std::string& name) {
    for (const GoalTask& goal : GOAL_LIST) {
        if (goal.name == name) {
            return &goal;
        }
    }
    return nullptr;
}

// 请求识别板一识别，并等待直接服务调用返回结果。
bool requestBoard1Vision(double timeout_sec, Board1Result* result) {
    if (g_use_mock_data) {
        if (result != nullptr) {
            *result = makeMockBoard1Result();
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

// 请求识别板二识别，并等待直接服务调用返回结果。
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
        ROS_ERROR("识别板一识别失败");
        return false;
    }
    board1_result.delivery_slot = std::max(1, std::min(4, board1_result.delivery_slot));

    if (!board1_result.has_a && !board1_result.has_b && !board1_result.has_c) {
        ROS_WARN("识别板一没有返回任何 A/B/C 取样窗口");
        return false;
    }

    ROS_INFO("识别板一结果：A=%d，B=%d，C=%d，delivery_slot=%d，sample_count=%d",
             board1_result.has_a,
             board1_result.has_b,
             board1_result.has_c,
             board1_result.delivery_slot,
             board1_result.sample_count);

    std::vector<std::string> pickup_route;
    if (board1_result.has_a) {
        pickup_route.push_back("pickup_A");
    }
    if (board1_result.has_c) {
        pickup_route.push_back("pickup_C");
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
    pnh.param("use_mock_data", g_use_mock_data, false);
    pnh.param("mock_navigation", g_mock_navigation, false);
    pnh.param("max_rounds", g_max_rounds, 0);
    pnh.param("board1_detection_service", board1_service, board1_service);
    pnh.param("board2_detection_service", board2_service, board2_service);

    MoveBaseClient move_client("move_base", true);
    ros::Subscriber image_sub = nh.subscribe("/camera/image_raw", 1, snapshotCB);
    g_board1_client = nh.serviceClient<move_nav::Board1Decode>(board1_service);
    g_board2_client = nh.serviceClient<move_nav::Board2Decode>(board2_service);

    if (!g_mock_navigation) {
        ROS_INFO("等待 move_base action server...");
        move_client.waitForServer();
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
            return 1;
        }

        ++completed_rounds;
        ROS_INFO("第 %d 轮任务完成", completed_rounds);
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("控制节点停止，已完成 %d 轮任务", completed_rounds);
    return 0;
}
