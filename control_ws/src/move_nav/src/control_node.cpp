#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tf2/LinearMath/Quaternion.h"
#include <vector>
#include <string>
#include <algorithm>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <sstream>
#include <std_msgs/String.h>
#include <string.h>
// --------------  用户可改宏  --------------
#ifndef SAVE_DIR
#define SAVE_DIR  "/home/zinn/snapshots/"   // 末尾斜杠别丢,要修改
#endif
enum TaskType { People = 0, Car, NoTask };
static std::atomic<int> g_idx(0);  // 自动编号
volatile TaskType pending_task = NoTask;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher g_task_request_pub;

// 当前目标点下标
size_t current_point = 0;

struct GoalTask {
    double x;
    double y;
    double yaw;
    TaskType task_type;
};

// 预定义目标点列表 {x坐标, y坐标, 航向角(yaw), 到点触发任务类型}
const std::vector<GoalTask> GOAL_LIST = {
    {3.38, 0.14, 1.57, NoTask},
    {3.44, 1.31, 3.14, NoTask},
    {2.62, 1.30, -1.57, People},
    {2.60, 1.25, 1.57, People},
    {1.82, 1.24, 1.57, NoTask},
    {1.82, 3.45, 3.14, NoTask},
    {0.59, 3.45, 3.14, NoTask},
    {0.64, 2.70, 3.14, Car},
    {0.64, 0.12, 3.14, NoTask},
    {0.05, -0.02, 0.0, NoTask}
};

const char* taskTypeToString(TaskType task_type) {
    if (task_type == People) {
        return "people_detection";
    }
    if (task_type == Car) {
        return "license_plate_recognition";
    }
    return "none";
}

void publishTaskRequest(TaskType task_type, size_t goal_index, const std::string& image_path) {
    std_msgs::String req_msg;
    std::ostringstream oss;
    oss << "task_type=" << taskTypeToString(task_type)
        << ";goal_index=" << goal_index
        << ";image_path=" << image_path
        << ";timestamp=" << ros::Time::now().toSec();
    req_msg.data = oss.str();
    g_task_request_pub.publish(req_msg);
    ROS_INFO("发布任务请求: %s", req_msg.data.c_str());
}

void taskResultCB(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("收到开发者实现节点返回结果: %s", msg->data.c_str());
}

void snapshotCB(const sensor_msgs::ImageConstPtr& msg) {
    if (pending_task == NoTask) {
        return;
    }

    try 
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        std::string fname = std::string(SAVE_DIR) + std::to_string(g_idx++) + ".jpg";

        if (cv::imwrite(fname, cv_ptr->image)) {
            ROS_INFO("Save %s", fname.c_str());
            publishTaskRequest(pending_task, current_point - 1, fname);
        } else {
            ROS_ERROR("Fail to save %s", fname.c_str());
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    pending_task = NoTask;  // 重置标志位
}

/**
  * @brief 移动机器人到指定目标点
  * 
  * @param client move_base action客户端
  **/
void movetoPoint(move_base_msgs::MoveBaseGoal goal, MoveBaseClient &client)
{
    ros::Rate rate(10);
    
    client.sendGoal(goal);//发送目标点 
    while (client.getState()!=actionlib::SimpleClientGoalState::ACTIVE)
    {
        ros::spinOnce();
        rate.sleep();
    }

    while (client.getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
    {        
        ros::spinOnce();
        rate.sleep();
    }
    //更新计数
    ROS_INFO("Point%zu reach!",current_point);
    ++current_point;
    ros::Duration(0.1).sleep();
    // 到达特定目标点后触发任务（由开发者实现具体处理）
    const TaskType task = GOAL_LIST[current_point - 1].task_type;
    if (task != NoTask) {
        ROS_INFO("Trigger task at goal %zu: %s", current_point - 1, taskTypeToString(task));
        pending_task = task;
    }

    client.cancelGoal();
}  

move_base_msgs::MoveBaseGoal toMove(int goal_n)
{
    double x = GOAL_LIST[goal_n].x;
    double y = GOAL_LIST[goal_n].y;
    double yaw = GOAL_LIST[goal_n].yaw;
    ROS_INFO("Moving to point %zu: (%.2f, %.2f, %.2f)", current_point, x, y, yaw);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;//2d导航z恒为0
    
    tf2::Quaternion q;
    q.setRPY(0,0,yaw);   
    goal.target_pose.pose.orientation.w = q.getW();
    goal.target_pose.pose.orientation.z = q.getZ();

    return goal;
}

int main(int argc, char* argv[]){
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"nav_move");
    ros::NodeHandle nh;
    MoveBaseClient MoveClient("move_base",true);
    ros::Subscriber sub = nh.subscribe("/camera/image_raw",1,snapshotCB);
    ros::Subscriber result_sub = nh.subscribe("smartcommunity/task_result", 10, taskResultCB);
    g_task_request_pub = nh.advertise<std_msgs::String>("smartcommunity/task_request", 10);

    ROS_INFO("=== 导航系统启动 ===");
    // 等待action服务器启动
    ROS_INFO("Waiting for move_base action server...");
    MoveClient.waitForServer();
    ROS_INFO("Connected to move_base action server");

    if(GOAL_LIST.empty()){
        ROS_ERROR("No navigation points defined");
        return 1;
    }
    ROS_INFO("开始导航，总共 %zu 个目标点", GOAL_LIST.size());
    ROS_INFO("视觉任务已抽象为 smartcommunity/task_request，开发者可自行订阅并实现。");
    ROS_INFO("结果回传订阅: smartcommunity/task_result");
    for(int i = 0; i < GOAL_LIST.size(); i++){
        movetoPoint(toMove(i),MoveClient);
        ros::spinOnce();
    }

    ROS_INFO("=== 导航完成 ===");
    ROS_INFO("导航任务结束。");
    ROS_INFO("如需业务统计，请在外部任务处理节点中实现并回传。");
    return 0;
}