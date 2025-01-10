#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include "OsqpEigen/OsqpEigen.h"
#include <geometry_msgs/Point.h>
#include <chrono>

// 定义路径格式
typedef std::vector<Eigen::Vector2d> Path;

// Step 4: 自行实现轨迹生成类
class TrajectoryGenerator {
    // 轨迹规划的目标是根据A*算法给出的无碰撞路径，计算轨迹航点（控制点），从而生成一条以时间参数化的平滑轨迹，可以用于控制移动机器人跟踪
    // 本次作业中，我们要求生成一条分段多项式轨迹，即每段轨迹均为一个多项式函数
    // 你可以选择使用多项式、B样条、贝塞尔曲线、MINCO等多种轨迹基函数实现
    // 每段轨迹的连接处需要满足一定的连续性条件，如位置连续、速度连续、加速度连续等，这将构成轨迹优化的主要约束条件
    // 轨迹的初始和终止状态为到达指定位置，速度、加速度等状态均为0
    // 优化目标可以是最小化轨迹的加加速度（jerk）或加加加速度（snap），请自行选择合适的优化目标及多项式阶数
    // 本次作业对轨迹段的时间选择不做进一步要求，可以自行选择固定时间或自定义策略生成时间分配
    // 可以任意选用求解器，如qpOASES、OSQP-Eigen等，也可以自行实现闭式解法
public:
    // TrajectoryGenerator() = default;

        // your code
    // 轨迹优化函数，接受原始路径点并返回优化后的路径
    // std::vector<Eigen::Vector2d> optimizeTrajectory(const std::vector<Eigen::Vector2d>& path_points) {
    //     std::vector<Eigen::Vector2d> optimized_path = path_points;

    //     // 这里进行轨迹优化的处理，举个例子，优化算法可能是基于平滑化路径
    //     // 例如使用简单的插值或拟合方法来优化路径

    //     // 例如：每隔一定的间隔优化路径
    //     for (auto& point : optimized_path) {
    //         point += Eigen::Vector2d(0.1, 0.1);  // 这是一个简单的示例，假设优化是增加一定的偏移
    //     }
    //     return optimized_path;
    // }
    TrajectoryGenerator(const std::vector<Eigen::Vector2d>& path_points, const int& num_samples)
        : path_(path_points), num_samples_(num_samples) {};
    TrajectoryGenerator() = default;

    // 基函数计算
    double BasisFun(const int& i, const int& k, const double& t,
                    const std::vector<double>& knots) {
        if (k == 1) {
            return (t >= knots[i] && t < knots[i+1]) ? 1.0 : 0.0;
        }
        double left_molecule = t - knots[i];
        double left_denominator = knots[i + k] - knots[i];
        double right_molecule = knots[i + k + 1] - t;
        double right_denominator = knots[i + k + 1] - knots[i + 1];

        double left_coeff, right_coeff;
        // 计算基函数时，如果分子为０，整个分数为０，如果分母为０，分母设置为１
        if (left_denominator == 0) {
            if (left_molecule == 0) {
                left_coeff = 0;
            }
            left_coeff = left_molecule / 1.;
        }
        else {
            left_coeff = left_molecule / left_denominator;
        }

        if (right_denominator == 0) {
            if (right_molecule == 0) {
                right_coeff = 0;
            }
            right_coeff = right_molecule / 1.;
        }
        else {
            right_coeff = right_molecule / right_denominator;
        }

        return left_coeff * BasisFun(i, k - 1, t, knots) + 
                right_coeff * BasisFun(i + 1, k - 1, t, knots);
    }

    // 生成节点向量
    std::vector<double> GenerateKnots(const int& k, 
                                           const int& num_control_points) {
        std::vector<double> knots(num_control_points + k + 1);
        for (int i = 0; i < k; i++) {
            knots[i] = 0.0;
        }
        for (int i = k; i <= num_control_points; i++) {
            knots[i] = static_cast<double>(i - k + 1) / (num_control_points - k + 1);
        }
        for (int i = num_control_points + 1; i < num_control_points + k + 1; i++) {
            knots[i] = 1.;
        }

        return knots;
    }

    // 轨迹优化处理
    std::vector<Eigen::Vector2d> Process() {
        // 记录优化开始时间
        auto start_time = std::chrono::steady_clock::now();

        int k = 4;
        size_t num_control_point = path_.size();

        std::vector<Eigen::Vector2d> optimized_path;

        ROS_INFO("Starting trajectory optimization...");

        // 在样本点上进行优化
        std::vector<double> knots = GenerateKnots(k, num_control_point);

        // 将优化后的轨迹点转为Eigen::Vector2d并添加到返回路径中
        for (int j = 0; j < num_samples_; j++) {
            Eigen::Vector2d point(0.0, 0.0);
            double t = static_cast<double>(j) / num_samples_*
                    (knots[knots.size() - 1] - knots[0]) + knots[0];
            for (int i = 0; i < num_control_point; i++) {
                double basis = BasisFun(i, k, t, knots);
                point[0] += basis * path_[i][0];
                point[1] += basis * path_[i][1];
            }
            optimized_path.push_back(point);
        }

        // 计算优化所用的时间并输出
        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> duration = end_time - start_time;
        ROS_INFO("Trajectory optimization completed in %.2f seconds.", duration.count());


        return optimized_path;
    }

private:
    std::vector<Eigen::Vector2d> path_;    // 原始路径点
    // std::vector<double> knots;            // 节点向量
    int num_samples_;                      // 样本点数

};
class TrajectoryNode {
public:
    TrajectoryNode() {
        // 初始化 ROS 节点
        ros::NodeHandle nh;

        // 订阅A*规划器发布的路径话题
        path_sub_ = nh.subscribe("path", 1, &TrajectoryNode::pathCallback, this);

        // 发布优化后的路径
        path_pub_ = nh.advertise<nav_msgs::Path>("optimized_path", 1);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        // 提取A*路径中的所有点
        std::vector<Eigen::Vector2d> path_points;
        for (const auto& pose : msg->poses) {
            Eigen::Vector2d point(pose.pose.position.x, pose.pose.position.y);
            path_points.push_back(point);
        }

        // 使用路径点进行轨迹优化
        // std::vector<Eigen::Vector2d> optimized_path = trajectory_optimizer_.optimizeTrajectory(path_points);
        // 设置样本点数量（即优化后的路径点数量）
        int num_samples = 2000;

        // 创建BSpline对象
        TrajectoryGenerator spline(path_points, num_samples);

        // 使用 Process 方法进行路径优化
        ROS_INFO("Starting to optimize the trajectory...");

        // 打印原始路径点
        ROS_INFO("Original path:");
        for (const auto& point : path_points) {
            ROS_INFO("Original Point: (%.2f, %.2f)", point.x(), point.y());
        }

        // 进行路径优化
        std::vector<Eigen::Vector2d> optimized_path = spline.Process();

        // 打印优化后的路径点
        ROS_INFO("Optimized path:");
        for (const auto& point : optimized_path) {
            ROS_INFO("Optimized Point: (%.2f, %.2f)", point.x(), point.y());
        }

        // 发布优化后的路径
        ROS_INFO("Publishing the optimized trajectory...");


        // 将优化后的路径发布到 "optimized_path" 话题
        publishOptimizedPath(optimized_path);
    }

    // 发布优化后的路径
    void publishOptimizedPath(const std::vector<Eigen::Vector2d>& optimized_path) {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";  // 使用map坐标系
        path_msg.header.stamp = ros::Time::now();

        for (const auto& point : optimized_path) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = 0.0;  // 设为平面路径
            path_msg.poses.push_back(pose);
        }

        // 发布优化后的路径
        path_pub_.publish(path_msg);
    }

private:
    ros::Subscriber path_sub_;  // 路径订阅器
    ros::Publisher path_pub_;   // 优化路径发布器
    // TrajectoryGenerator trajectory_optimizer_;  // 轨迹优化器
};

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "trajectory_optimizer_node");


    // 创建 TrajectoryNode 对象，启动 ROS 节点
    TrajectoryNode trajectory_node;

    // 进入 ROS 节点的主循环
    ros::spin();

    return 0;
}