#ifndef MY_LOCAL_PLANNER_H
#define MY_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <angles/angles.h>
#include <vector>
#include <string>

namespace my_local_planner {

class MyLocalPlanner : public nav_core::BaseLocalPlanner {
public:
    MyLocalPlanner();
    
    // 必须实现的接口
    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
    bool isGoalReached() override;
    
    // 数据获取接口 - 供算法使用
    const std::vector<geometry_msgs::PoseStamped>& getGlobalPlan() const { return global_plan_; }
    const std::vector<geometry_msgs::PoseStamped>& getLocalPlan() const { return local_plan_; }
    const geometry_msgs::PoseStamped& getRobotPose() const { return current_robot_pose_; }
    const geometry_msgs::PoseStamped& getGoalPose() const { return current_goal_; }
    
    // 代价地图访问接口
    costmap_2d::Costmap2DROS* getCostmapROS() { return costmap_ros_; }
    const costmap_2d::Costmap2DROS* getCostmapROS() const { return costmap_ros_; }
    
    // 工具函数
    double getYawFromOrientation(const geometry_msgs::Quaternion& quat) const;
    double getDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2) const;
    double getAngleDifference(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2) const;
    
    // 速度发布接口
    void publishVelocity(const geometry_msgs::Twist& cmd_vel);
    
    // 算法函数
    geometry_msgs::Twist computeMyAlgorithm();

private:
    // 初始化标志
    bool initialized_;
    
    // ROS组件
    tf2_ros::Buffer* tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    ros::Publisher vel_pub_, path_pub_;
    
    // 数据存储
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    std::vector<geometry_msgs::PoseStamped> local_plan_;
    geometry_msgs::PoseStamped current_robot_pose_;
    geometry_msgs::PoseStamped current_goal_;
    base_local_planner::OdometryHelperRos odom_helper_;
    
    // 参数
    double max_vel_x_, min_vel_x_;
    double max_vel_y_, min_vel_y_;
    double max_vel_theta_, min_vel_theta_;
    double yaw_goal_tolerance_, xy_goal_tolerance_;
    
    // 内部方法
    bool updateRobotPose();
    bool transformGlobalPlan();
    void publishLocalPlan();
};

} // namespace my_local_planner

#endif