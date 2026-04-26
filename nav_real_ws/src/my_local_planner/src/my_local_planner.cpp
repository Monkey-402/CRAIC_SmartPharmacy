#include <my_local_planner/my_local_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <cmath>

namespace my_local_planner {

int path_num=0;
double path_distance_x=0,path_distance_y=0;
MyLocalPlanner::MyLocalPlanner() : 
    initialized_(false),
    tf_(nullptr),
    costmap_ros_(nullptr) {
}

void MyLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        
        // 获取参数path_num
        ros::NodeHandle private_nh("~/" + name);
        
        private_nh.param("max_vel_x", max_vel_x_, 0.5);
        private_nh.param("min_vel_x", min_vel_x_, -0.5);
        private_nh.param("max_vel_y", max_vel_y_, 0.5);
        private_nh.param("min_vel_y", min_vel_y_, -0.5);
        private_nh.param("max_vel_theta", max_vel_theta_, 1.0);
        private_nh.param("min_vel_theta", min_vel_theta_, -1.0);
        private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
        private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
        
        // 初始化里程计帮助器
        std::string odom_topic;
        private_nh.param("odom_topic", odom_topic, std::string("odom"));
        odom_helper_.setOdomTopic(odom_topic);
        
        // 发布器
        vel_pub_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        path_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
        
        ROS_INFO("MyLocalPlanner initialized successfully");
        ROS_INFO("Velocity limits - X: [%.2f, %.2f], Y: [%.2f, %.2f], Theta: [%.2f, %.2f]", 
                min_vel_x_, max_vel_x_, min_vel_y_, max_vel_y_, min_vel_theta_, max_vel_theta_);
        
        initialized_ = true;
    }
}


// ========== 工具函数 ==========

double MyLocalPlanner::getYawFromOrientation(const geometry_msgs::Quaternion& quat) const {
    tf2::Quaternion tf_quat;
    tf2::fromMsg(quat, tf_quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    return yaw;
}

double MyLocalPlanner::getDistance(const geometry_msgs::PoseStamped& pose1, 
                                  const geometry_msgs::PoseStamped& pose2) const {
    double dx = pose1.pose.position.x - pose2.pose.position.x;
    double dy = pose1.pose.position.y - pose2.pose.position.y;
    return sqrt(dx*dx + dy*dy);
}

double MyLocalPlanner::getAngleDifference(const geometry_msgs::PoseStamped& pose1, 
                                         const geometry_msgs::PoseStamped& pose2) const {
    double yaw1 = getYawFromOrientation(pose1.pose.orientation);
    double yaw2 = getYawFromOrientation(pose2.pose.orientation);
    return angles::shortest_angular_distance(yaw1, yaw2);
}

// 计算点到直线的距离
double pointToLineDistance(const geometry_msgs::Point& point, 
    const geometry_msgs::Point& line_start, 
    const geometry_msgs::Point& line_end) {

    double line_vec_x = line_end.x - line_start.x;
    double line_vec_y = line_end.y - line_start.y;

    double point_vec_x = point.x - line_start.x;
    double point_vec_y = point.y - line_start.y;

    double line_length_sq = line_vec_x * line_vec_x + line_vec_y * line_vec_y;

    if (line_length_sq == 0.0) {
    return sqrt(point_vec_x * point_vec_x + point_vec_y * point_vec_y);
    }

    double t = (point_vec_x * line_vec_x + point_vec_y * line_vec_y) / line_length_sq;

    t = std::max(0.0, std::min(1.0, t));

    double projection_x = line_start.x + t * line_vec_x;
    double projection_y = line_start.y + t * line_vec_y;

    double dx = point.x - projection_x;
    double dy = point.y - projection_y;

    return sqrt(dx * dx + dy * dy);
}

// 主函数：计算两个路径点之间的直线与它们之间路径点的最大距离
double calculateMaxDistanceBetweenPoints(const std::vector<geometry_msgs::PoseStamped>& path,
                  int index1, int index2) {

    if (index2 - index1 == 1) {
        return 0.0;
    }

    const geometry_msgs::Point& start_point = path[index1].pose.position;
    const geometry_msgs::Point& end_point = path[index2].pose.position;

    double max_distance = 0.0;

    for (int i = index1 + 1; i < index2; i++) {
    const geometry_msgs::Point& current_point = path[i].pose.position;
    double distance = pointToLineDistance(current_point, start_point, end_point);
    max_distance = std::max(max_distance, distance);

    ROS_DEBUG("Point %d (%.3f, %.3f) distance to line: %.6f", 
    i, current_point.x, current_point.y, distance);
    }

    ROS_DEBUG("Max distance between points %d and %d: %.6f", index1, index2, max_distance);

    return max_distance;
}

bool MyLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR("MyLocalPlanner not initialized");
        return false;
    }
    
    global_plan_ = plan;
    path_num=1;
    
    if (!plan.empty()) {
        current_goal_ = plan.back();
        ROS_INFO("Global plan updated: %zu waypoints", plan.size());
        ROS_INFO("Goal position: (%.3f, %.3f)", 
                current_goal_.pose.position.x, 
                current_goal_.pose.position.y);
        
        path_distance_x=current_goal_.pose.position.x-current_robot_pose_.pose.position.x;
        path_distance_y=current_goal_.pose.position.y-current_robot_pose_.pose.position.y;
    }
    
    return true;
}

bool MyLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if (!initialized_) {
        ROS_ERROR("MyLocalPlanner not initialized");
        return false;
    }
    
    // 步骤1: 更新所有必要的数据
    if (!updateRobotPose()) {
        ROS_WARN("Failed to update robot pose");
        return false;
    }
    
    if (!transformGlobalPlan()) {
        ROS_WARN("Failed to transform global plan");
        return false;
    }
    
    // 步骤2: 发布局部路径用于可视化
    publishLocalPlan();
    
    // 步骤3: 调用你的算法
    cmd_vel = computeMyAlgorithm();
    
    // 步骤4: 发布速度命令
    publishVelocity(cmd_vel);
    
    return true;
}

bool MyLocalPlanner::isGoalReached() {
    if (!initialized_ || global_plan_.empty()) {
        return false;
    }
    
    if (!updateRobotPose()) {
        return false;
    }
    
    // 检查位置容差
    double distance = getDistance(current_robot_pose_, current_goal_);
    if (distance > xy_goal_tolerance_) {
        return false;
    }
    
    // 检查朝向容差
    double angle_diff = getAngleDifference(current_robot_pose_, current_goal_);
    if (fabs(angle_diff) > yaw_goal_tolerance_) {
        return false;
    
    }
    
    ROS_INFO("Goal reached!");
    return true;
}

// ========== 数据更新方法 ==========

bool MyLocalPlanner::updateRobotPose() {
    bool success = costmap_ros_->getRobotPose(current_robot_pose_);
    if (success) {
        current_robot_pose_.header.stamp = ros::Time::now();
    }
    return success;
}

bool MyLocalPlanner::transformGlobalPlan() {
    local_plan_.clear();
    
    if (global_plan_.empty()) {
        return false;
    }
    
    const std::string global_frame = costmap_ros_->getGlobalFrameID();
    
    try {
        geometry_msgs::TransformStamped transform = tf_->lookupTransform(
            current_robot_pose_.header.frame_id, global_frame, ros::Time(0));
        
        // 转换所有路径点到局部坐标系
        for (const auto& global_pose : global_plan_) {
            geometry_msgs::PoseStamped local_pose;
            tf2::doTransform(global_pose, local_pose, transform);
            local_plan_.push_back(local_pose);
        }
        
        return true;
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("Transform failure: %s", ex.what());
        return false;
    }
}

void MyLocalPlanner::publishLocalPlan() {
    if (path_pub_.getNumSubscribers() > 0) {
        nav_msgs::Path local_path;
        local_path.header.frame_id = costmap_ros_->getGlobalFrameID();
        local_path.header.stamp = ros::Time::now();
        local_path.poses = local_plan_;
        path_pub_.publish(local_path);
    }
}


// ========== 速度发布接口 ==========

void MyLocalPlanner::publishVelocity(const geometry_msgs::Twist& cmd_vel) {
    // 速度限制检查
    geometry_msgs::Twist limited_cmd_vel = cmd_vel;
    
    limited_cmd_vel.linear.x = std::max(min_vel_x_, std::min(max_vel_x_, cmd_vel.linear.x));
    limited_cmd_vel.linear.y = std::max(min_vel_y_, std::min(max_vel_y_, cmd_vel.linear.y));
    limited_cmd_vel.angular.z = std::max(min_vel_theta_, std::min(max_vel_theta_, cmd_vel.angular.z));
    ROS_INFO("limited_cmd_vel output: lin_x=%.3f, lin_y=%.3f, ang_z=%.3f", 
        limited_cmd_vel.linear.x, limited_cmd_vel.linear.y, limited_cmd_vel.angular.z);
    // 发布速度命令
    vel_pub_.publish(limited_cmd_vel);
    
    ROS_DEBUG_THROTTLE(0.5, "Velocity published: lin_x=%.3f, lin_y=%.3f, ang_z=%.3f", 
                      limited_cmd_vel.linear.x, limited_cmd_vel.linear.y, limited_cmd_vel.angular.z);
}

// ========== 算法函数 ==========
geometry_msgs::Twist MyLocalPlanner::computeMyAlgorithm() {
    geometry_msgs::Twist cmd_vel;
    
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    
    // 获取所有必要数据
    const auto& global_plan = getGlobalPlan();
    const auto& local_plan = getLocalPlan();
    const auto& robot_pose = getRobotPose();
    const auto& goal_pose = getGoalPose();
    
    // 检查数据是否有效
    if (local_plan.empty()) {
        ROS_WARN("Local plan is empty!");
        return cmd_vel;
    }
    
    // 机器人当前位置和朝向
    double robot_x = robot_pose.pose.position.x;
    double robot_y = robot_pose.pose.position.y;
    double robot_yaw = getYawFromOrientation(robot_pose.pose.orientation);
    if(getDistance(robot_pose,global_plan[path_num])<=0.1){
        int path_num_nxt=path_num;
        while(calculateMaxDistanceBetweenPoints(global_plan,path_num,path_num_nxt+1)<=0.01 && path_num<global_plan.size()-1)
        {
            path_num_nxt++;
        }
        path_num=path_num_nxt;
        ROS_INFO("path_num: %d", path_num_nxt);
    }

    if(path_num>=global_plan.size())path_num=global_plan.size()-1;
    // 目标位置和朝向
    double goal_x = global_plan[path_num].pose.position.x;
    double goal_y = global_plan[path_num].pose.position.y;
    //  goal_x = goal_pose.pose.position.x;
    //  goal_y = goal_pose.pose.position.y;
    double goal_yaw = getYawFromOrientation(goal_pose.pose.orientation);
    
    const auto& local_target = local_plan[0];
    double target_x = local_target.pose.position.x;
    double target_y = local_target.pose.position.y;
    

    ROS_INFO("=== Algorithm Input Data ===");
    ROS_INFO("Robot: (%.3f, %.3f, %.3f rad)", robot_x, robot_y, robot_yaw);
    ROS_INFO("Goal: (%.3f, %.3f, %.3f rad)", goal_x, goal_y, goal_yaw);
    ROS_INFO("Local target: (%.3f, %.3f)", target_x, target_y);
    ROS_INFO("Global path points: %zu", global_plan.size());
    ROS_INFO("Local path points: %zu", local_plan.size());

    // ROS_INFO("path_length:%.3f,%.3f",path_distance_x,(goal_x - robot_x));

    // 手动计算全局坐标差
    double dx_global = goal_x - robot_x;
    double dy_global = goal_y - robot_y;
    // if(abs(dx_global)>abs(path_distance_x)/2)dx_global=path_distance_x-dx_global;
    // if(abs(dy_global)>abs(path_distance_y)/2)dy_global=path_distance_y-dy_global;
    // ROS_INFO("vx:%.3f,vy:%.3f",dx_global,dy_global);

    // 转换到机器人坐标系
    double dx_local = dx_global * cos(robot_yaw) + dy_global * sin(robot_yaw);
    double dy_local = -dx_global * sin(robot_yaw) + dy_global * cos(robot_yaw);
    double dyaw_local = goal_yaw - robot_yaw;
    cmd_vel.linear.x = dx_local * 3.0;
    cmd_vel.linear.y = dy_local * 3.0;

    if(abs(getDistance(goal_pose,robot_pose)<=0.1))
        cmd_vel.angular.z = dyaw_local * 4.0;
    
    // ROS_INFO("Global diff: (%.3f, %.3f) -> Local: (%.3f, %.3f)", 
    //         dx_global, dy_global, dx_local, dy_local);
    // =============================================
    // 算法结束
    // =============================================
    cmd_vel.linear.x = std::max(min_vel_x_, std::min(max_vel_x_, cmd_vel.linear.x));
    cmd_vel.linear.y = std::max(min_vel_y_, std::min(max_vel_y_, cmd_vel.linear.y));
    cmd_vel.angular.z = std::max(min_vel_theta_, std::min(max_vel_theta_, cmd_vel.angular.z));

    ROS_INFO("Algorithm output: lin_x=%.3f, lin_y=%.3f, ang_z=%.3f", 
              cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
    
    return cmd_vel;
}

} // namespace my_local_planner

// 注册插件
PLUGINLIB_EXPORT_CLASS(my_local_planner::MyLocalPlanner, nav_core::BaseLocalPlanner)