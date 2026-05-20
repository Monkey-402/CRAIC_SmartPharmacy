#include <algorithm>
#include <atomic>
#include <clocale>
#include <sstream>
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

#include "move_nav/TaskRequest.h"
#include "move_nav/TaskResult.h"

/*
 * 本模板只生成 cpp，不修改 CMakeLists.txt，也不创建 msg 文件。
 * 这个节点只负责“任务调度 + 导航动作”，不直接做二维码/OCR/语音合成。
 *
 * 一轮任务流程：
 *   1. 到识别板一(board1_scan)拍照，向视觉节点请求 board1_decode。
 *   2. 视觉节点返回本轮二维码中的汇总结果：A/B/C 是否有样本、目的地、样本数。
 *   3. 控制节点根据汇总结果生成取样路线，再依次到 A/B/C 取样窗口并播报。
 *   4. 到识别板二(board2_scan)拍照，向视觉节点请求 board2_decode。
 *   5. 根据识别板二结果播报并等待，然后去 1/2/3/4 化验窗口送样。
 *
 * 建议的最小消息格式：
 *
 * TaskRequest.msg  #请求消息格式
 *   string task_id     #任务编号
 *   string task_type   #任务类型，例如 "board1_decode" 或 "board2_decode"，
 *                      #视觉节点通过这个字段区分识别板一/识别板二任务。
 *   string image_path  #图片路径
 *
 * TaskResult.msg   #结果消息格式
 *   string task_id  #任务编号（和请求消息中的 task_id写成一样就可以了）
 *   bool has_a     # A 窗口是否有样本，有就置1
 *   bool has_b     # B 窗口是否有样本
 *   bool has_c     # C 窗口是否有样本
 *   int32 delivery_slot   # 送达目标点 1=血常规，2=体液，3=免疫检测，4=激素检验
 *   int32 sample_count     #样本数量
 * 
 *   bool lab_open
 *   int32 wait_seconds
 *   string speech_text
 *
 * 视觉节点订阅 /smartcommunity/task_request，只处理：
 *   task_type == "board1_decode"
 *   task_type == "board2_decode"
 *
 * 视觉节点处理完成后发布 /smartcommunity/task_result。
 */

#ifndef SAVE_DIR
#define SAVE_DIR "/home/zinn/snapshots/"
#endif

//定义别名
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct GoalTask {
    double x;
    double y;
    double yaw;
    std::string name;  // 任务点名称，用于把识别结果映射到具体导航点。
};

// 这些坐标是模板占位值。实车或仿真运行前，需要用对应地图重新取点。
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

ros::Publisher g_task_request_pub;// 任务请求发布
ros::Publisher g_audio_play_pub;// 语音请求发布

static std::atomic<int> g_img_idx(0);// 图像序号计数器
static std::atomic<int> g_task_idx(0);// 任务序号计数器

// 运行模式与配置
bool g_use_mock_data = false;
bool g_mock_navigation = false;
int g_max_rounds = 0;

size_t current_point = 0;// 导航点索引

bool g_take_photo = false;

std::string g_pending_task_id;// 当前待处理的视觉任务 ID
std::string g_pending_task_type;// 当前待处理的视觉任务类型，例如 "board1_decode" 或 "board2_decode"。

// 最近一次视觉任务结果。通过 task_id 匹配，避免把旧结果当成当前任务结果。
std::string g_last_result_task_id;
move_nav::TaskResult g_last_result;// 最近一次视觉任务结果

// 生成模拟视觉识别结果
move_nav::TaskResult makeMockTaskResult(const std::string& task_id,
                                        const std::string& task_type) {
    move_nav::TaskResult result;
    result.task_id = task_id;

    if (task_type == "board1_decode") {
        result.has_a = true;
        result.has_b = true;
        result.has_c = true;
        result.delivery_slot = 1;
        result.sample_count = 3;
    } else if (task_type == "board2_decode") {
        result.lab_open = true;
        result.wait_seconds = 0;
        result.speech_text = "化验区空闲中，请快速通过";
    }

    return result;
}

// 生成唯一任务 ID，用于匹配任务请求和异步返回的任务结果。
std::string makeTaskId(const std::string& task_type) {
    std::ostringstream oss;
    oss << task_type << "_" << ros::Time::now().toNSec() << "_" << g_task_idx++;
    return oss.str();
}

// 发布视觉任务请求，视觉节点通过 task_type 判断识别板一/识别板二任务。
void publishTaskRequest(const move_nav::TaskRequest& request) {
    g_task_request_pub.publish(request);
    ROS_INFO("Publish task request: task_id=%s, task_type=%s, image_path=%s",
             request.task_id.c_str(),
             request.task_type.c_str(),
             request.image_path.c_str());
}

// 语音播放占位函数。后续接真实语音 API 时，只需要改这里。
void playAudioFile(const std::string& audio_file) {
    if (audio_file.empty()) {
        ROS_WARN("Audio file path is empty, skip playback");
        return;
    }

    std_msgs::String msg;
    msg.data = audio_file;
    g_audio_play_pub.publish(msg);
    ROS_INFO("Play audio file: %s", audio_file.c_str());
}

// 接收视觉节点返回的结果，并缓存最近一次结果供 waitForTaskResult 查询。
void taskResultCB(const move_nav::TaskResult::ConstPtr& msg) {
    if (msg->task_id.empty()) {
        ROS_WARN("Ignore task result without task_id");
        return;
    }

    g_last_result_task_id = msg->task_id;
    g_last_result = *msg;
    ROS_INFO("Receive task result: task_id=%s", msg->task_id.c_str());
}

// 等待指定 task_id 的视觉任务结果，超时则返回 false。
bool waitForTaskResult(const std::string& task_id,
                       double timeout_sec,
                       move_nav::TaskResult* result) {
    ros::Rate rate(20);
    const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_sec);

    while (ros::ok() && ros::Time::now() < deadline) {
        ros::spinOnce();
        // 通过 task_id 匹配，避免把旧结果当成当前任务结果。
        if (g_last_result_task_id == task_id) {
            if (result != nullptr) {
                *result = g_last_result;
            }
            return true;
        }
        rate.sleep();
    }

    ROS_WARN("Wait task result timeout: task_id=%s", task_id.c_str());
    return false;
}

// 相机图像回调；只有 g_take_photo 打开时，才保存下一帧图片并发布视觉任务。
void snapshotCB(const sensor_msgs::ImageConstPtr& msg) {
    if (!g_take_photo) {
        return;
    }

    try {
        // 只在任务需要拍照时保存一帧，避免持续写磁盘。
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        const std::string image_path =
            std::string(SAVE_DIR) + std::to_string(g_img_idx++) + ".jpg";

        if (!cv::imwrite(image_path, cv_ptr->image)) {
            ROS_ERROR("Failed to save image: %s", image_path.c_str());
            g_take_photo = false;
            return;
        }

        ROS_INFO("Saved image: %s", image_path.c_str());
        move_nav::TaskRequest request;
        request.task_id = g_pending_task_id;
        request.task_type = g_pending_task_type;
        request.image_path = image_path;
        publishTaskRequest(request);
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    g_take_photo = false;
}

// 将内部导航点 GoalTask 转换成 move_base 可执行的 MoveBaseGoal。
move_base_msgs::MoveBaseGoal toMove(const GoalTask& goal_task) {
    ROS_INFO("Moving to %s: (%.2f, %.2f, %.2f)",
             goal_task.name.c_str(), goal_task.x, goal_task.y, goal_task.yaw);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = goal_task.x;
    goal.target_pose.pose.position.y = goal_task.y;
    goal.target_pose.pose.position.z = 0.0;

    // move_base 使用四元数表达朝向；这里从平面 yaw 角转换。
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, goal_task.yaw);
    goal.target_pose.pose.orientation.x = q.getX();
    goal.target_pose.pose.orientation.y = q.getY();
    goal.target_pose.pose.orientation.z = q.getZ();
    goal.target_pose.pose.orientation.w = q.getW();
    return goal;
}

// 发送导航目标并等待到达；成功返回 true，导航失败返回 false。
bool movetoPoint(const GoalTask& goal_task, MoveBaseClient& client) {
    if (g_mock_navigation) {
        ROS_INFO("[MOCK_NAV] Reached %s: (%.2f, %.2f, %.2f)",
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
            ROS_ERROR("Navigation failed: %s, state=%s",
                      goal_task.name.c_str(), state.toString().c_str());
            client.cancelGoal();
            return false;
        }
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Point%zu reached: %s", current_point, goal_task.name.c_str());
    ++current_point;
    client.cancelGoal();
    ros::Duration(0.1).sleep();
    return true;
}

// 按任务点名称查找对应导航点，例如 board1_scan、pickup_A、deliver_1。
const GoalTask* findGoalByName(const std::string& name) {
    for (const GoalTask& goal : GOAL_LIST) {
        if (goal.name == name) {
            return &goal;
        }
    }
    return nullptr;
}

// 设置拍照标志
// task_type 可为 board1_decode 或 board2_decode。
bool requestVisionTaskAtCurrentPoint(const std::string& task_type,
                                     double timeout_sec,
                                     move_nav::TaskResult* result) {
    g_pending_task_id = makeTaskId(task_type);  // task_id 是这一次识别请求的编号，不是样本编号。
    g_pending_task_type = task_type;

    // 如果启用 mock 数据，则不拍照，直接发布任务请求和假结果，方便调试流程和导航。
    if (g_use_mock_data) {
        move_nav::TaskRequest request;
        request.task_id = g_pending_task_id;
        request.task_type = g_pending_task_type;
        request.image_path = "mock://no-camera";
        publishTaskRequest(request);

        if (result != nullptr) {
            *result = makeMockTaskResult(g_pending_task_id, g_pending_task_type);
        }
        ROS_INFO("[MOCK_DATA] Use fake result: task_id=%s, task_type=%s",
                 g_pending_task_id.c_str(), g_pending_task_type.c_str());
        return true;
    }

    g_take_photo = true;

    ROS_INFO("Trigger vision task: task_id=%s, task_type=%s",
             g_pending_task_id.c_str(), g_pending_task_type.c_str());
    const bool ok = waitForTaskResult(g_pending_task_id, timeout_sec, result);
    if (!ok) {
        g_take_photo = false;
    }
    return ok;
}

// 执行一轮完整配送任务：识别板一 -> 取样 -> 识别板二 -> 送样。
bool runOneQrMission(MoveBaseClient& move_client) {
    ROS_INFO("========== Start one QR mission ==========");

    // 先到识别板一拍照并解析，得到本轮任务的 A/B/C 窗口状态、目的地和样本数。
    const GoalTask* board1_goal = findGoalByName("board1_scan");
    if (board1_goal == nullptr) {
        ROS_ERROR("GOAL_LIST has no board1_scan goal");
        return false;
    }

    if (!movetoPoint(*board1_goal, move_client)) {
        return false;
    }

    move_nav::TaskResult board1_result;
    if (!requestVisionTaskAtCurrentPoint("board1_decode", 15.0, &board1_result)) {
        ROS_ERROR("Failed to decode board1");
        return false;
    }
    board1_result.delivery_slot = std::max(1, std::min(4, board1_result.delivery_slot));

    if (!board1_result.has_a && !board1_result.has_b && !board1_result.has_c) {
        ROS_WARN("Board1 returned no A/B/C source window");
        return false;
    }

    ROS_INFO("Board1: A=%d, B=%d, C=%d, delivery_slot=%d, sample_count=%d",
             board1_result.has_a,
             board1_result.has_b,
             board1_result.has_c,
             board1_result.delivery_slot,
             board1_result.sample_count);

    // 一次性取完当前二维码中的所有样本
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

    // 依次到 C/A/B 取样窗口取样，并记录取样窗口名称用于后续播报。
    std::string pickup_windows;
    for (const std::string& goal_name : pickup_route) {
        const GoalTask* goal = findGoalByName(goal_name);
        if (goal == nullptr) {
            ROS_ERROR("GOAL_LIST has no pickup goal: %s", goal_name.c_str());
            return false;
        }
        if (!movetoPoint(*goal, move_client)) {
            return false;
        }

        ros::Duration(1.5).sleep();// 停留1.5秒
        const std::string window_name = goal_name.substr(goal_name.size() - 1);
        if (!pickup_windows.empty()) {
            pickup_windows += "、";
        }
        pickup_windows += window_name;
        ROS_INFO("Sample picked: source_slot=%s", window_name.c_str());
    }

    const char* sample_type = "静脉血样本";
    if (board1_result.delivery_slot == 2) {
        sample_type = "唾液样本";
    } else if (board1_result.delivery_slot == 3) {
        sample_type = "组织样本";
    } else if (board1_result.delivery_slot == 4) {
        sample_type = "血浆样本";
    }

    ROS_INFO("Pickup speech: 取到%s窗口的%s", pickup_windows.c_str(), sample_type);
    playAudioFile("/path/to/pickup_summary.mp3");

    // 进入化验区前先处理识别板二的空闲/忙碌提示。
    const GoalTask* board2_goal = findGoalByName("board2_scan");
    if (board2_goal == nullptr) {
        ROS_ERROR("GOAL_LIST has no board2_scan goal");
        return false;
    }
    if (!movetoPoint(*board2_goal, move_client)) {
        return false;
    }

    // 请求识别板二的视觉任务，获取化验区状态和等待时间等信息，并根据结果播报提示语。
    move_nav::TaskResult board2_result;
    if (!requestVisionTaskAtCurrentPoint("board2_decode", 15.0, &board2_result)) {
        ROS_WARN("Board2 vision task failed or timed out, defaulting to lab open");
        board2_result.lab_open = true;
        board2_result.wait_seconds = 0;
        board2_result.speech_text = "化验区空闲中，请快速通过";
    }

    board2_result.wait_seconds = std::max(0, board2_result.wait_seconds);
    if (board2_result.speech_text.empty()) {
        board2_result.speech_text =
            board2_result.lab_open ? "化验区空闲中，请快速通过" : "化验区忙碌中，请等待";
    }
    if (!board2_result.lab_open && board2_result.wait_seconds <= 0) {
        board2_result.wait_seconds = 5;
    }

    ROS_INFO("Board2 speech: %s", board2_result.speech_text.c_str());
    playAudioFile("/path/to/board2_notice.mp3");
    if (board2_result.wait_seconds > 0) {
        ROS_INFO("Lab area busy, wait %d seconds before passing", board2_result.wait_seconds);
        ros::Duration(board2_result.wait_seconds).sleep();
    }

    // 当前二维码中的样本都送到同一个化验窗口，因此只需要去一个送样点。
    const std::string delivery_goal_name =
        "deliver_" + std::to_string(board1_result.delivery_slot);
    const GoalTask* delivery_goal = findGoalByName(delivery_goal_name);
    if (delivery_goal == nullptr) {
        ROS_ERROR("GOAL_LIST has no delivery goal: %s", delivery_goal_name.c_str());
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
    ROS_INFO("Samples delivered: delivery_slot=%d, count=%d",
             board1_result.delivery_slot, board1_result.sample_count);
    ROS_INFO("Delivery speech: 到达%s，样本数为%d",
             delivery_window, board1_result.sample_count);
    playAudioFile("/path/to/delivery_notice.mp3");

    playAudioFile("/path/to/mission_done.mp3");
    ROS_INFO("========== One QR mission finished ==========");
    return true;
}

// ROS 节点入口：初始化通信接口，连接 move_base，并循环执行配送任务。
int main(int argc, char* argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "yaofang_control_msg_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("use_mock_data", g_use_mock_data, false);
    pnh.param("mock_navigation", g_mock_navigation, false);
    pnh.param("max_rounds", g_max_rounds, 0);

    // 图像、识别结果、任务请求三类接口分别连接相机、视觉节点和语音/视觉节点。
    MoveBaseClient move_client("move_base", true);
    ros::Subscriber image_sub = nh.subscribe("/camera/image_raw", 1, snapshotCB);
    ros::Subscriber result_sub = nh.subscribe("/smartcommunity/task_result", 10, taskResultCB);
    g_task_request_pub = nh.advertise<move_nav::TaskRequest>("/smartcommunity/task_request", 10);
    g_audio_play_pub = nh.advertise<std_msgs::String>("/smartcommunity/audio_play", 10);

    ROS_INFO("=== Message-based yaofang control node started ===");
    ROS_INFO("Params: use_mock_data=%d, mock_navigation=%d, max_rounds=%d",
             g_use_mock_data, g_mock_navigation, g_max_rounds);
    if (!g_mock_navigation) {
        ROS_INFO("Waiting for move_base action server...");
        move_client.waitForServer();
        ROS_INFO("Connected to move_base action server");
    } else {
        ROS_INFO("[MOCK_NAV] Skip move_base action server");
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
        ROS_INFO("Mission round %d finished", completed_rounds);
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("Control node stopped after %d completed rounds", completed_rounds);
    return 0;
}
