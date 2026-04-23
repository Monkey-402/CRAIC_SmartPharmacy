#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tf2/LinearMath/Quaternion.h"
#include <vector>
#include <string>
#include <filesystem>
#include <algorithm>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <atomic>
#include "yolo_onnx_ros/ObjectDetection.h"
#include "license_plate_ocr/LicensePlateOCR.h"
#include<string.h>
// --------------  用户可改宏  --------------
#ifndef SAVE_DIR
#define SAVE_DIR  "/home/zinn/snapshots/"   // 末尾斜杠别丢,要修改
#endif
typedef enum{People=0,Car,No} TakePhotoFlag;
static std::atomic<int> g_idx(0);              // 自动编号
//触发拍照标志位
// volatile bool takePhoto = false;
volatile TakePhotoFlag takePhoto = No;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//统计变量
static int A_Street_People = 0;                // 第一次人物识别人数
static int B_Street_People = 0;                // 第二次人物识别人数
static int A_Street_People_Outside = 0;        // A街外来人口数目
static int B_Street_People_Outside = 0;        // B街外来人口数目
static bool A_Street_Processed = false;        // A街是否已处理
static bool B_Street_Processed = false;        // B街是否已处理

// 停车场相关变量
static int ParkNum = 0;                        // 停车场数量
static std::map<int, std::string> parking_lot_plates; // 停车场序号到车牌号的映射

// 需要拍照的 goal 下标（0-based，与 GOAL_LIST 对应）
static const std::vector<size_t> PHOTO_IDX = {2,3};                //人物识别
static const std::vector<size_t> LICENSE_PLATE_IDX = {7};          //车牌识别
//当前目标点下标
size_t current_point = 0;

// 预定义目标点列表 {x坐标, y坐标, 航向角(yaw)}
const std::vector<std::tuple<double, double, double>> GOAL_LIST = {
    {  3.38,  0.14, 1.57},   
    { 3.44, 1.31, 3.14},
    { 2.62,  1.30, -1.57},//拍照1
    { 2.60,  1.25, 1.57}, //拍照2     
    { 1.82,  1.24, 1.57},
    { 1.82,  3.45, 3.14},
    { 0.59,  3.45, 3.14},      
    { 0.64,  2.70, 3.14},//车牌 
    { 0.64,  0.12, 3.14},
    { 0.05,  -0.02, 0}
};

/**
 * @brief 处理人物识别结果并更新统计信息
 * @param srv_response YOLO服务响应
 * @param is_A_street 是否为A街识别
 */
void processPeopleDetection(const yolo_onnx_ros::ObjectDetection::Response& srv_response, bool is_A_street) {
    int local_count = 0;
    int outside_count = 0;
    
    // 统计本地人口和外来人口
    for (size_t i = 0; i < srv_response.class_names.size(); ++i) {
        if (srv_response.class_names[i] == "0" || srv_response.class_names[i] == "peopleInside") {  // 本地人口
            local_count += srv_response.class_counts[i];
        } else if (srv_response.class_names[i] == "1" || srv_response.class_names[i] == "peopleOutside") {  // 外来人口
            outside_count += srv_response.class_counts[i];
        }
    }
    
    int total_count = local_count + outside_count;
    
    if (is_A_street) {
        A_Street_People = total_count;
        A_Street_People_Outside = outside_count;
        A_Street_Processed = true;
        ROS_INFO("A街识别完成: 总人数=%d, 其中外来人口=%d", total_count, outside_count);
    } else {
        B_Street_People = total_count;
        B_Street_People_Outside = outside_count;
        B_Street_Processed = true;
        ROS_INFO("B街识别完成: 总人数=%d, 其中外来人口=%d", total_count, outside_count);
    }
    
    // 如果两次识别都已完成，显示汇总信息
    if (A_Street_Processed && B_Street_Processed) {
        int People_Sum = A_Street_People + B_Street_People;
        ROS_INFO("==============================================");
        ROS_INFO("社区内共有%d人，其中A街发现%d人，B街发现%d人，发现%d名非社区人员在A街，%d名非社区人员在B街，图片已保存。", 
                 People_Sum, A_Street_People, B_Street_People, 
                 A_Street_People_Outside, B_Street_People_Outside);
        ROS_INFO("==============================================");
    }
}

/**
 * @brief 处理车牌识别结果
 * @param srv_response 车牌识别服务响应
  * @param parking_lot_id 停车场序号（从1开始）
 */
void processLicensePlateDetection(const license_plate_ocr::LicensePlateOCR::Response& srv_response, int parking_lot_id) {
    ROS_INFO("=== 开始处理停车场%d的车牌识别结果 ===", parking_lot_id);
    
    if (srv_response.plate_texts.empty()) {
        ROS_WARN("停车场%d未检测到任何车牌", parking_lot_id);
        parking_lot_plates[parking_lot_id] = "未识别到车牌";
    } else {
        // 只取第一个车牌（每个停车场只有一辆车）
        std::string plate_text = srv_response.plate_texts[0];
        float confidence = srv_response.confidences[0];
        
        parking_lot_plates[parking_lot_id] = plate_text;
        
        // ROS_INFO("停车场%d检测到车牌: %s, 置信度: %.4f", 
        //          parking_lot_id, plate_text.c_str(), confidence);
        
        // 输出要求的格式
        ROS_INFO("%d号停车场车牌号为%s", parking_lot_id, plate_text.c_str());
        
        // 如果有多个车牌，只记录第一个并警告
        if (srv_response.plate_texts.size() > 1) {
            ROS_WARN("停车场%d检测到多个车牌，但每个停车场应只有一辆车，只取第一个车牌", parking_lot_id);
        }
    }
    
    ROS_INFO("=== 停车场%d车牌识别处理结束 ===\n", parking_lot_id);
}

//获取当前目标点对应的停车场序号
int getParkingLotId(size_t current_goal) {
    auto it = std::find(LICENSE_PLATE_IDX.begin(), LICENSE_PLATE_IDX.end(), current_goal);
    if (it != LICENSE_PLATE_IDX.end()) {
        // 返回从1开始的序号
        return std::distance(LICENSE_PLATE_IDX.begin(), it) + 1;
    }
    return -1;
}

void snapshotCB(const sensor_msgs::ImageConstPtr& msg){
    if(takePhoto == No) return;
    try 
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        std::string fname = std::string(SAVE_DIR) + std::to_string(g_idx++) + ".jpg";
        if (cv::imwrite(fname, cv_ptr->image)) {
            ROS_INFO("Save %s", fname.c_str());

            ros::NodeHandle nh;
            if (takePhoto == People) 
            {
                // 判断是A街还是B街的识别
                bool is_A_street = (current_point - 1 == PHOTO_IDX[0]); // 第一个拍照点是A街
                // 调用 YOLO 服务端
                ros::ServiceClient client = nh.serviceClient<yolo_onnx_ros::ObjectDetection>("detect_objects");
                yolo_onnx_ros::ObjectDetection srv;
                srv.request.image = *msg; // 将图像消息传递给服务请求
                if (client.call(srv)) 
                {
                    if (srv.response.success) 
                    {
                        ROS_INFO("Detection successful!");
                        #if 0
                        ROS_INFO("Message: %s", srv.response.message.c_str());
                        ROS_INFO("Detected classes conf: ");
                        for (const auto& conf : srv.response.confidences) {
                            ROS_INFO(" %.2f", conf);
                        }
                        ROS_INFO("Class names: ");
                        for (const auto& name : srv.response.class_names) {
                            ROS_INFO("  %s", name.c_str());
                        }
                        ROS_INFO("Class counts: ");
                        for (const auto& count : srv.response.class_counts) 
                        {
                            ROS_INFO("  %d", count);
                        }
                        #endif
                        // 处理人物识别结果
                        processPeopleDetection(srv.response, is_A_street);
                    } 
                    else 
                    {
                        ROS_ERROR("Detection failed: %s", srv.response.message.c_str());
                    }   
                } 
                else 
                {
                    ROS_ERROR("Failed to call service /detect_objects");
                }
            } 
            else if (takePhoto == Car) 
            {
                // 获取当前停车场序号
                int parking_lot_id = getParkingLotId(current_point - 1);
                if (parking_lot_id == -1) 
                {
                    ROS_ERROR("当前位置不是停车场识别点");
                    takePhoto = No;
                    return;
                }
                
                ROS_INFO("开始停车场%d的车牌识别...", parking_lot_id);
                ros::ServiceClient client = nh.serviceClient<license_plate_ocr::LicensePlateOCR>("license_plate_ocr");
                
                if (!client.waitForExistence(ros::Duration(5.0))) 
                {
                    ROS_ERROR("车牌识别服务不可用");
                    takePhoto = No;
                    return;
                }
                
                license_plate_ocr::LicensePlateOCR srv;
                srv.request.image = *msg;

                if (client.call(srv)) 
                {                   
                    ROS_INFO("车牌识别状态：%s", srv.response.status_message.c_str());
                    
                    // 处理车牌识别结果
                    processLicensePlateDetection(srv.response, parking_lot_id);
                    
                } 
                else 
                {
                    ROS_ERROR("调用车牌识别服务失败");
                }
            }
            else
            {
                ROS_ERROR("Failed to call service /license_plate_ocr");
            }
            
        } 
        else
        {
            ROS_ERROR("Fail to save %s", fname.c_str());
        }
    } 
    catch (cv_bridge::Exception& e) 
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    takePhoto = No;  // 重置标志位
}

/**
  * @brief 移动机器人到指定目标点
  * 
  * @param x 目标点x坐标
  * @param y 目标点y坐标
  * @param z 目标点z坐标
  * @param quat_x 四元数x分量
  * @param quat_y 四元数y分量
  * @param quat_z 四元数z分量
  * @param quat_w 四元数w分量
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
    //到达特定目标点后触发拍照
    if (std::find(PHOTO_IDX.begin(), PHOTO_IDX.end(), current_point - 1) != PHOTO_IDX.end()) 
    {
        ROS_INFO("Trigger photo at goal %zu", current_point - 1);
        takePhoto = People;  // 触发 YOLO 服务端
    } 
    else if (std::find(LICENSE_PLATE_IDX.begin(), LICENSE_PLATE_IDX.end(), current_point - 1) != LICENSE_PLATE_IDX.end()) 
    {
        ROS_INFO("Trigger photo at goal %zu for license plate OCR", current_point - 1);
        takePhoto = Car;  // 触发车牌识别服务端
    }

    client.cancelGoal();
}  

move_base_msgs::MoveBaseGoal toMove(int goal_n)
{
    double x = std::get<0>(GOAL_LIST[goal_n]);
    double y = std::get<1>(GOAL_LIST[goal_n]);
    double yaw = std::get<2>(GOAL_LIST[goal_n]);
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

    // 初始化统计信息
    A_Street_People = 0;
    B_Street_People = 0;
    A_Street_People_Outside = 0;
    B_Street_People_Outside = 0;
    A_Street_Processed = false;
    B_Street_Processed = false;
    // 初始化停车场信息
    ParkNum = LICENSE_PLATE_IDX.size();  // 停车场数量等于车牌识别点的数量
    parking_lot_plates.clear();

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
    ROS_INFO("人物识别点: %zu, %zu", PHOTO_IDX[0], PHOTO_IDX[1]);
    ROS_INFO("车牌识别点数量: %zu", LICENSE_PLATE_IDX.size());
    for(int i = 0; i < GOAL_LIST.size(); i++){
        movetoPoint(toMove(i),MoveClient);
        ros::spinOnce();
    }

    // 最终统计信息显示
    ROS_INFO("=== 导航完成 ===");
    ROS_INFO("==============================================");
    ROS_INFO("最终统计结果:");
    
    // 人物统计
    if (A_Street_Processed || B_Street_Processed) 
    {
        int People_Sum = A_Street_People + B_Street_People;
        ROS_INFO("社区内共有%d人，其中A街发现%d人，B街发现%d人，发现%d名非社区人员在A街，%d名非社区人员在B街，图片已保存。", 
                 People_Sum, A_Street_People, B_Street_People, 
                 A_Street_People_Outside, B_Street_People_Outside);
    }
    else 
    {
        ROS_WARN("未完成任何人物识别统计");
    }
    
    // 车牌识别结果
    if (!parking_lot_plates.empty()) 
    {
        ROS_INFO("车牌识别结果:");
        for (const auto& pair : parking_lot_plates) {
            ROS_INFO("  %d号停车场车牌号为%s", pair.first, pair.second.c_str());
        }
    } 
    else 
    {
        ROS_WARN("未完成任何车牌识别");
    }
    
    ROS_INFO("==============================================");
    ROS_INFO("导航任务结束!");
    return 0;
}
