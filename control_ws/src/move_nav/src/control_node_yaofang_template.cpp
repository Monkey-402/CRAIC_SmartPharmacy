#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdlib>
#include <map>
#include <sstream>
#include <string>
#include <vector>


#ifndef SAVE_DIR
#define SAVE_DIR "/home/zinn/snapshots/"
#endif

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

enum TaskType {
    Board1Scan = 0, // 到识别板一拍照，获取样本窗口、样本类别、化验区窗口
    Board2Scan,     // 到点拍照，识别板2，判断化验区是否可进入
    PickupSample,   // 到点即认为获取样本成功
    DeliverSample,  // 到点即认为配送样本成功
    NoTask
};

struct GoalTask {
    double x;
    double y;
    double yaw;
    TaskType task_type;
    std::string name;  // 业务点名称，用来和二维码结果做映射
};

struct SampleOrder {
    std::string sample_id;       // 样本编号，例如 S01
    std::string source_slot;     // 体检区取样窗口，例如 A、B、C
    std::string sample_type;     // 样本类别，例如 blood、saliva、tissue、plasma
    std::string delivery_slot;   // 化验区目标窗口，例如 1、2、3、4
    bool picked = false;
    bool delivered = false;
};

const std::vector<GoalTask> GOAL_LIST = {
    {0.00, 0.00, 0.00, NoTask,        "home"},
    {1.00, 0.50, 1.57, Board1Scan,    "board1_scan"},
    {2.20, 0.90, 0.00, PickupSample,  "pickup_A"},
    {2.20, 1.70, 0.00, PickupSample,  "pickup_B"},
    {2.20, 2.50, 0.00, PickupSample,  "pickup_C"},
    {3.10, 1.60, 1.57, Board2Scan,    "board2_scan"},
    {4.00, 0.80, 3.14, DeliverSample, "deliver_1"},
    {4.00, 1.60, 3.14, DeliverSample, "deliver_2"},
    {4.00, 2.40, 3.14, DeliverSample, "deliver_3"},
    {4.00, 3.20, 3.14, DeliverSample, "deliver_4"}
};

const std::vector<SampleOrder> MOCK_QR_ORDERS = {
    {"S01", "A", "blood", "1", false, false},
    {"S02", "B", "blood", "1", false, false},
    {"S03", "C", "blood", "1", false, false}
};

ros::Publisher g_task_request_pub;

static std::atomic<int> g_img_idx(0);
static std::atomic<int> g_task_idx(0);

size_t current_point = 0;

volatile bool take_photo = false;
std::string pending_task_id;
std::string pending_task_type;
std::string pending_goal_name;

std::string last_result_task_id;
std::string last_result_data;

//trim：去掉字符串两边的空格
std::string trim(const std::string& s) {
    size_t begin = 0;
    while (begin < s.size() && std::isspace(static_cast<unsigned char>(s[begin]))) {
        ++begin;
    }

    size_t end = s.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
        --end;
    }
    return s.substr(begin, end - begin);
}


std::vector<std::string> split(const std::string& s, char delim) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        item = trim(item);
        if (!item.empty()) {
            out.push_back(item);
        }
    }
    return out;
}

std::map<std::string, std::string> parseKeyValue(const std::string& text) {
    std::map<std::string, std::string> kv;
    for (const std::string& item : split(text, ';')) {
        const size_t pos = item.find('=');
        if (pos == std::string::npos) {
            continue;
        }
        kv[trim(item.substr(0, pos))] = trim(item.substr(pos + 1));
    }
    return kv;
}

std::string makeTaskId(const std::string& task_type) {
    std::ostringstream oss;
    oss << task_type << "_" << ros::Time::now().toNSec() << "_" << g_task_idx++;
    return oss.str();
}

//任务类型转换为字符串
const char* taskTypeToString(TaskType task_type) {
    if (task_type == Board1Scan) {
        return "board1_decode";
    }
    if (task_type == Board2Scan) {
        return "board2_decode";
    }
    if (task_type == PickupSample) {
        return "pickup_sample";
    }
    if (task_type == DeliverSample) {
        return "deliver_sample";
    }
    return "none";
}

//发布任务请求
void publishTaskRequest(const std::string& task_id,
                        const std::string& task_type,
                        const std::map<std::string, std::string>& fields) {
    std_msgs::String msg;
    std::ostringstream oss;
    oss << "task_id=" << task_id
        << ";task_type=" << task_type
        << ";timestamp=" << ros::Time::now().toSec();

    for (const auto& field : fields) {
        oss << ";" << field.first << "=" << field.second;
    }

    msg.data = oss.str();
    g_task_request_pub.publish(msg);
    ROS_INFO("发布任务请求: %s", msg.data.c_str());
}

void publishSpeechNotice(const std::string& text) {
    publishTaskRequest(
        makeTaskId("speech_notice"),
        "speech_notice",
        {{"text", text}});
}

void taskResultCB(const std_msgs::String::ConstPtr& msg) {
    const std::map<std::string, std::string> kv = parseKeyValue(msg->data);
    const auto it = kv.find("task_id");

    if (it == kv.end()) {
        ROS_WARN("收到没有 task_id 的结果，忽略: %s", msg->data.c_str());
        return;
    }

    last_result_task_id = it->second;
    last_result_data = msg->data;
    ROS_INFO("收到任务结果: %s", msg->data.c_str());
}

bool waitForTaskResult(const std::string& task_id,
                       double timeout_sec,
                       std::string* result_data) {
    ros::Rate rate(20);
    const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_sec);

    while (ros::ok() && ros::Time::now() < deadline) {

        ros::spinOnce();
        if (last_result_task_id == task_id) {
            if (result_data != nullptr) {
                *result_data = last_result_data;
            }
            return true;
        }
        rate.sleep();
    }

    ROS_WARN("等待任务结果超时: task_id=%s", task_id.c_str());
    return false;
}


void snapshotCB(const sensor_msgs::ImageConstPtr& msg) {
    if (!take_photo) {
        return;
    }

    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        const std::string image_path =
            std::string(SAVE_DIR) + std::to_string(g_img_idx++) + ".jpg";

        if (!cv::imwrite(image_path, cv_ptr->image)) {
            ROS_ERROR("图片保存失败: %s", image_path.c_str());
            take_photo = false;
            return;
        }

        ROS_INFO("图片已保存: %s", image_path.c_str());

        publishTaskRequest(
            pending_task_id,
            pending_task_type,
            {
                {"goal_name", pending_goal_name},
                {"image_path", image_path}
            });
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    take_photo = false;
}


bool requestImageTaskAtCurrentPoint(const std::string& task_type,
                                    const std::string& goal_name,
                                    double timeout_sec,
                                    std::string* result) {
    pending_task_id = makeTaskId(task_type);
    pending_task_type = task_type;
    pending_goal_name = goal_name;
    take_photo = true;

    ROS_INFO("触发视觉任务 %s，等待视觉节点回传结果...", task_type.c_str());
    return waitForTaskResult(pending_task_id, timeout_sec, result);
}

// 在当前点触发板一视觉任务并等待回传
bool requestBoard1DecodeAtCurrentPoint(const std::string& goal_name,
                                       std::string* board1_result) {
    return requestImageTaskAtCurrentPoint("board1_decode", goal_name, 15.0, board1_result);
}

// 在当前点触发板二视觉任务并等待回传
bool requestBoard2DecodeAtCurrentPoint(const std::string& goal_name,
                                       std::string* board2_result) {
    return requestImageTaskAtCurrentPoint("board2_decode", goal_name, 15.0, board2_result);
}

// GoalTask → move_base 目标
move_base_msgs::MoveBaseGoal toMove(const GoalTask& goal_task) {
    ROS_INFO("Moving to %s: (%.2f, %.2f, %.2f)",
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

// 发送导航目标并等待到达
bool movetoPoint(const GoalTask& goal_task, MoveBaseClient& client) {
    ros::Rate rate(10);
    move_base_msgs::MoveBaseGoal goal = toMove(goal_task);

    client.sendGoal(goal);

    while (ros::ok() && client.getState() != actionlib::SimpleClientGoalState::ACTIVE) {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok() && client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        const actionlib::SimpleClientGoalState state = client.getState();
        if (state == actionlib::SimpleClientGoalState::ABORTED ||
            state == actionlib::SimpleClientGoalState::REJECTED ||
            state == actionlib::SimpleClientGoalState::LOST) {
            ROS_ERROR("导航失败: %s, state=%s",
                      goal_task.name.c_str(), state.toString().c_str());
            client.cancelGoal();
            return false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Point%zu reach: %s, task=%s",
             current_point, goal_task.name.c_str(), taskTypeToString(goal_task.task_type));
    ++current_point;
    client.cancelGoal();
    ros::Duration(0.1).sleep();
    return true;
}

// 按名称查找 GOAL_LIST 中的导航点
const GoalTask* findGoalByName(const std::string& name) {
    for (const GoalTask& goal : GOAL_LIST) {
        if (goal.name == name) {
            return &goal;
        }
    }
    return nullptr;
}

// 解析板一回传中的 samples 字段，生成 SampleOrder 列表
std::vector<SampleOrder> parseOrdersFromBoard1Result(const std::string& result_data) {
    const std::map<std::string, std::string> kv = parseKeyValue(result_data);

    std::string sample_text;
    auto it = kv.find("samples");
    if (it != kv.end()) {
        sample_text = it->second;
    }

    if (sample_text.empty()) {
        ROS_WARN("识别板一结果中没有 samples 字段，使用 MOCK_QR_ORDERS");
        return MOCK_QR_ORDERS;
    }

    std::vector<SampleOrder> orders;
    for (const std::string& token : split(sample_text, ',')) {
        const size_t at_pos = token.find('@');
        const size_t colon_pos = token.find(':');
        const size_t arrow_pos = token.find("->");
        if (at_pos == std::string::npos ||
            colon_pos == std::string::npos ||
            arrow_pos == std::string::npos ||
            at_pos > colon_pos ||
            colon_pos > arrow_pos) {
            ROS_WARN("样本字段格式不正确，跳过: %s", token.c_str());
            continue;
        }

        SampleOrder order;
        order.sample_id = trim(token.substr(0, at_pos));
        order.source_slot = trim(token.substr(at_pos + 1, colon_pos - at_pos - 1));
        order.sample_type = trim(token.substr(colon_pos + 1, arrow_pos - colon_pos - 1));
        order.delivery_slot = trim(token.substr(arrow_pos + 2));
        orders.push_back(order);
    }

    if (orders.empty()) {
        ROS_WARN("识别板一解析后样本为空，使用 MOCK_QR_ORDERS");
        return MOCK_QR_ORDERS;
    }
    return orders;
}

// 体检区窗口 → pickup 导航点名称
std::string pickupGoalNameForSource(const std::string& source_slot) {
    if (source_slot.empty()) {
        return "pickup_A";
    }

    const char area = static_cast<char>(
        std::toupper(static_cast<unsigned char>(source_slot[0])));
    if (area == 'B') {
        return "pickup_B";
    }
    if (area == 'C') {
        return "pickup_C";
    }
    return "pickup_A";
}

// 化验区窗口编号 → deliver 导航点名称
std::string deliveryGoalNameForSlot(const std::string& delivery_slot) {
    if (delivery_slot == "2" || delivery_slot == "P2") {
        return "deliver_2";
    }
    if (delivery_slot == "3" || delivery_slot == "P3") {
        return "deliver_3";
    }
    if (delivery_slot == "4" || delivery_slot == "P4") {
        return "deliver_4";
    }
    return "deliver_1";
}

// 去重并保持原有顺序
std::vector<std::string> uniqueGoalNamesInOrder(const std::vector<std::string>& names) {
    std::vector<std::string> result;
    for (const std::string& name : names) {
        if (std::find(result.begin(), result.end(), name) == result.end()) {
            result.push_back(name);
        }
    }
    return result;
}

// 根据订单生成取样路线（同一体检区只取一次）
std::vector<std::string> buildPickupRoute(const std::vector<SampleOrder>& orders) {
    std::vector<std::string> route;
    for (const SampleOrder& order : orders) {
        route.push_back(pickupGoalNameForSource(order.source_slot));
    }
    return uniqueGoalNamesInOrder(route);
}

// 根据订单生成配送路线（同一化验窗口只送一次）
std::vector<std::string> buildDeliveryRoute(const std::vector<SampleOrder>& orders) {
    std::vector<std::string> route;
    for (const SampleOrder& order : orders) {
        route.push_back(deliveryGoalNameForSlot(order.delivery_slot));
    }
    return uniqueGoalNamesInOrder(route);
}

std::string sampleTypeName(const std::string& sample_type) {
    if (sample_type == "blood" || sample_type == "1") {
        return "静脉血样本";
    }
    if (sample_type == "saliva" || sample_type == "2") {
        return "唾液样本";
    }
    if (sample_type == "tissue" || sample_type == "3") {
        return "组织样本";
    }
    if (sample_type == "plasma" || sample_type == "4") {
        return "血浆样本";
    }
    return sample_type;
}

std::string deliveryWindowName(const std::string& delivery_slot) {
    if (delivery_slot == "1") {
        return "血常规窗口";
    }
    if (delivery_slot == "2") {
        return "体液窗口";
    }
    if (delivery_slot == "3") {
        return "免疫检测窗口";
    }
    if (delivery_slot == "4") {
        return "激素检验窗口";
    }
    return "化验窗口" + delivery_slot;
}

// 到达取样点时标记对应 source_slot 的订单为已取
void markPickedAtGoal(std::vector<SampleOrder> orders, const std::string& goal_name) {
    // 规则：获取样本时车身进入方框并明显停留（建议 1~2s）
    ros::Duration(1.0).sleep();

    for (SampleOrder& order : orders) {
        if (!order.picked && pickupGoalNameForSource(order.source_slot) == goal_name) {
            order.picked = true;
            ROS_INFO("到达取样点，样本获取成功: sample_id=%s, source_slot=%s",
                     order.sample_id.c_str(), order.source_slot.c_str());
            publishSpeechNotice("取到" + order.source_slot + "窗口中的" +
                                sampleTypeName(order.sample_type));
        }
    }
}


// 到达配送点时标记对应 delivery_slot 的订单为已送
void markDeliveredAtGoal(std::vector<SampleOrder> orders, const std::string& goal_name) {
    // 规则：送达样本时车身进入方框并明显停留（建议 1~2s）
    ros::Duration(1.0).sleep();

    int delivered_count = 0;
    std::string delivery_slot;
    for (SampleOrder& order : orders) {
        if (!order.delivered && deliveryGoalNameForSlot(order.delivery_slot) == goal_name) {
            order.delivered = true;
            ++delivered_count;
            delivery_slot = order.delivery_slot;
            ROS_INFO("到达配送点，样本配送成功: sample_id=%s, delivery_slot=%s",
                     order.sample_id.c_str(), order.delivery_slot.c_str());
        }
    }

    if (delivered_count > 0) {
        publishSpeechNotice("到达" + deliveryWindowName(delivery_slot) +
                            "，样本数为" + std::to_string(delivered_count));
    }
}

// 是否全部样本已取样
bool allPicked(const std::vector<SampleOrder>& orders) {
    for (const SampleOrder& order : orders) {
        if (!order.picked) {
            return false;
        }
    }
    return true;
}

// 是否全部样本已配送
bool allDelivered(const std::vector<SampleOrder>& orders) {
    for (const SampleOrder& order : orders) {
        if (!order.delivered) {
            return false;
        }
    }
    return true;
}

struct Board2Decision {
    bool can_pass = true;
    int wait_seconds = 0;
    std::string speech_text = "化验区空闲中，请快速通过";
};

// 解析板二回传，得到等待时间与播报文本
Board2Decision parseBoard2Decision(const std::string& board2_result) {
    Board2Decision decision;
    const std::map<std::string, std::string> kv = parseKeyValue(board2_result);

    auto text_it = kv.find("text");
    if (text_it != kv.end()) {
        decision.speech_text = text_it->second;
    }

    // wait_sec 兼容 wait_seconds
    auto wait_it = kv.find("wait_sec");
    if (wait_it == kv.end()) {
        wait_it = kv.find("wait_seconds");
    }
    if (wait_it != kv.end()) {
        decision.wait_seconds = std::max(0, std::atoi(wait_it->second.c_str()));
    }

    // lab_open 兼容 can_enter
    auto it = kv.find("lab_open");
    if (it == kv.end()) {
        it = kv.find("can_enter");
    }

    if (it == kv.end()) {
        ROS_WARN("识别板2结果中没有 lab_open/can_enter 字段，按空闲处理");
        return decision;
    }

    
    const std::string value = it->second;
    const bool open = value == "true" || value == "1" || value == "yes" || value == "open";
    decision.can_pass = true;

    // 规则：空闲则 3s 内尽快通过；忙碌则等待 n 秒后通过
    if (!open && decision.wait_seconds <= 0) {
        decision.wait_seconds = 5;
    }
    return decision;
}


// 识别板二并处理化验区空闲/忙碌逻辑
bool handleBoard2BeforeLabArea(MoveBaseClient& move_client) {
    const GoalTask* board2_goal = findGoalByName("board2_scan");
    if (board2_goal == nullptr) {
        ROS_ERROR("GOAL_LIST 中没有 board2_scan 点");
        return false;
    }

    if (!movetoPoint(*board2_goal, move_client)) {
        return false;
    }

    std::string board2_result;
    if (!requestBoard2DecodeAtCurrentPoint(board2_goal->name, &board2_result)) {
        ROS_WARN("识别板2暂未返回结果，按空闲处理");
        publishSpeechNotice("化验区空闲中，请快速通过");
        return true;
    }

    const Board2Decision decision = parseBoard2Decision(board2_result);
    publishSpeechNotice(decision.speech_text);

    if (decision.wait_seconds > 0) {
        ROS_INFO("识别板2提示化验区忙碌，等待 %d 秒后通过", decision.wait_seconds);
        ros::Duration(decision.wait_seconds).sleep();
    } else {
        ROS_INFO("识别板2提示化验区空闲，尽快通过");
    }
    return true;
}

// 单个二维码完整流程：板一 → 取样 → 板二 → 配送
bool runOneQrMission(MoveBaseClient& move_client) {
    ROS_INFO("开始药房任务");

    const GoalTask* board1_goal = findGoalByName("board1_scan");
    if (board1_goal == nullptr) {
        ROS_ERROR("GOAL_LIST 中没有 board1_scan 点");
        return false;
    }

    if (!movetoPoint(*board1_goal, move_client)) {
        return false;
    }

    std::string board1_result;
    std::vector<SampleOrder> orders;
    if (requestBoard1DecodeAtCurrentPoint(board1_goal->name, &board1_result)) {
        orders = parseOrdersFromBoard1Result(board1_result);
    } else {
        ROS_WARN("识别板一暂未返回，使用 MOCK_QR_ORDERS");
        orders = MOCK_QR_ORDERS;
    }

    ROS_INFO("识别板一二维码中共有 %zu 个样本，本次任务将一次性全部配送", orders.size());

    const std::vector<std::string> pickup_route = buildPickupRoute(orders);
    for (const std::string& goal_name : pickup_route) {
        const GoalTask* goal = findGoalByName(goal_name);
        if (goal == nullptr) {
            ROS_ERROR("GOAL_LIST 中没有取样点: %s", goal_name.c_str());
            return false;
        }
        
        if (!movetoPoint(*goal, move_client)) {
            return false;
        }
        markPickedAtGoal(orders, goal_name);
    }

    if (!allPicked(orders)) {
        ROS_ERROR("仍有样本未获取，停止配送");
        return false;
    }
    publishSpeechNotice("已获取当前二维码中的全部样本");

    if (!handleBoard2BeforeLabArea(move_client)) {
        return false;
    }

    const std::vector<std::string> delivery_route = buildDeliveryRoute(orders);
    for (const std::string& goal_name : delivery_route) {
        const GoalTask* goal = findGoalByName(goal_name);
        if (goal == nullptr) {
            ROS_ERROR("GOAL_LIST 中没有配送点: %s", goal_name.c_str());
            return false;
        }

        if (!movetoPoint(*goal, move_client)) {
            return false;
        }
        markDeliveredAtGoal(orders, goal_name);
    }

    if (!allDelivered(orders)) {
        ROS_ERROR("仍有样本未配送完成");
        return false;
    }

    ROS_INFO("药房任务完成");
    publishSpeechNotice("当前二维码中的全部样本配送完成");
    return true;
}

int main(int argc, char* argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "yaofang_control_node");
    ros::NodeHandle nh;

    MoveBaseClient move_client("move_base", true);
    ros::Subscriber image_sub = nh.subscribe("/camera/rgb/image_raw", 1, snapshotCB);
    ros::Subscriber result_sub = nh.subscribe("/smartcommunity/task_result", 10, taskResultCB);
    g_task_request_pub = nh.advertise<std_msgs::String>("/smartcommunity/task_request", 10);

    ROS_INFO("=== 智慧药房控制节点启动 ===");
    ROS_INFO("等待 move_base action server...");
    move_client.waitForServer();
    ROS_INFO("move_base action server 已连接");

    if (GOAL_LIST.empty()) {
        ROS_ERROR("GOAL_LIST 为空");
        return 1;
    }

    const bool ok = runOneQrMission(move_client);

    const GoalTask* home_goal = findGoalByName("home");
    if (home_goal != nullptr) {
        movetoPoint(*home_goal, move_client);
    }

    if (!ok) {
        ROS_ERROR("智慧药房任务失败");
        publishSpeechNotice("配送任务异常，请人工检查");
        return 1;
    }

    ROS_INFO("智慧药房任务完成");
    return 0;
}
