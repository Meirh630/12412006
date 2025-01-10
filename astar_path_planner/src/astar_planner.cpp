#include <ros/ros.h>
#include <utility>
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <Eigen/Dense>
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/Point.h>
#include "nav_msgs/Path.h"



struct Node {
    int x, y;        // 节点所在的网格坐标
    double g_cost;   // 从起点到当前节点的代价
    double h_cost;   // 从当前节点到终点的估计代价
    std::shared_ptr<Node> parent;    // 父节点，用于回溯路径

    Node(int x, int y, double g_cost, double h_cost, std::shared_ptr<Node> parent = nullptr)
            : x(x), y(y), g_cost(g_cost), h_cost(h_cost), parent(std::move(parent)) {}

    double f() const { return g_cost + h_cost; } // 总代价值

};
// 比较器，用于优先队列
struct cmp{
    bool operator()(std::shared_ptr<Node> a, std::shared_ptr<Node> b){
        return a->f() > b->f();
    }

};
struct GridMap {
    int width;
    int height;
    double map_max;
    double map_min;
    double grid_resolution;
    std::vector<std::vector<int>> grid; // 0: 空闲, 1: 占用

    GridMap(int w, int h, double map_min_, double map_max_, double res) : width(w), height(h), map_min(map_min_), map_max(map_max_), grid_resolution(res), grid(w, std::vector<int>(h, 0)) {}

    void markObstacle(double cx, double cy, double radius) {
        int grid_cx = std::round((cx - map_min) / grid_resolution);
        int grid_cy = std::round((cy - map_min) / grid_resolution);
        int grid_radius = std::round(radius / grid_resolution);
        // Step 1: 将圆形区域标记为占用
            // your code
        // finish
        // 遍历圆形区域内的所有网格
        for (int dx = -grid_radius; dx <= grid_radius; ++dx) {
            for (int dy = -grid_radius; dy <= grid_radius; ++dy) {
                int nx = grid_cx + dx;
                int ny = grid_cy + dy;

                // 检查是否在地图范围内
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    // 检查是否在圆形内 (勾股定理)
                    if (std::sqrt(dx * dx + dy * dy) <= grid_radius) {
                        grid[nx][ny] = 1; // 标记为占用
                    }
                }
            }
        }
    }
};
class AStarPlanner {
public:
    AStarPlanner(int width, int height, double m_min, double m_max, double res) : width_(width), height_(height), map_min_(m_min), map_max_(m_max), grid_resolution_(res), grid_map_(width, height, map_min_, map_max_, grid_resolution_), num_of_obs_(0) {

    }

    void setObstacle(double cx, double cy, double radius) {
        num_of_obs_++;
        grid_map_.markObstacle(cx, cy, radius);
    }

    void printGridMap(){
        for(int i = 0; i < width_; i++){
            for(int j = 0; j < height_; j++){
                std::cout<<grid_map_.grid[i][j]<<" ";
            }
            std::cout<<std::endl;
        }
        std::cout<<"num of obstacles: "<<num_of_obs_<<std::endl;
    }

    void printClosedList(const std::vector<std::vector<bool>>& closed_list) {
        std::cout << "Closed List: " << std::endl;
        for (int i = 0; i < closed_list.size(); ++i) {
            for (int j = 0; j < closed_list[i].size(); ++j) {
                std::cout << closed_list[i][j] << " ";  // 打印每个元素的值
            }
            std::cout << std::endl;  // 每一行打印完后换行
        }
    }

    void printGcostList(const std::vector<std::vector<double>>& g_cost_list) {
        std::cout << "Gcost List: " << std::endl;
        for (int i = 0; i < g_cost_list.size(); ++i) {
            for (int j = 0; j < g_cost_list[i].size(); ++j) {
                std::cout << g_cost_list[i][j] << " ";  // 打印每个元素的值
            }
            std::cout << std::endl;  // 每一行打印完后换行
        }
    }

    std::vector<Eigen::Vector2d> findPath(Eigen::Vector2d start, Eigen::Vector2d goal) {
        if(num_of_obs_ == 0){
            return {};
        }
        // 起点和终点转换为网格坐标
        auto gridStart = worldToGrid(start);
        auto gridGoal = worldToGrid(goal);

        // 开放列表和关闭列表
        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, cmp> open_list;
        std::vector<std::vector<bool>> closed_list(width_, std::vector<bool>(height_, false));

        // 起点加入开放列表
        open_list.push(std::make_shared<Node>(Node(gridStart.first, gridStart.second, 0.0, heuristic(gridStart, gridGoal))));
        // Step 3： 实现 A* 算法，搜索结束调用 reconstructPath 返回路径

            // 样例路径，用于给出路径形式，实现 A* 算法时请删除
                // std::vector<Eigen::Vector2d> path;
                // int num_points = 100; // 生成路径上的点数
                // for (int i = 0; i <= num_points; ++i) {
                //     double t = static_cast<double>(i) / num_points;
                //     Eigen::Vector2d point = start + t * (goal - start);
                //     path.push_back(point);
                // }
                // return path;
            // 注释结束
            // your code

        // finish

        // 如果没有找到路径，返回空路径
        std::vector<std::vector<double>> g_cost_list(width_, std::vector<double>(height_, 0));


        // A* 算法的核心循环
        while (!open_list.empty()) {
            // 取出代价最小的节点
            auto currentNode = open_list.top();
            open_list.pop();
            // std::cout << "Current Node: (" << currentNode->x << ", " << currentNode->y << ")"
            //           << " g_cost: " << currentNode->g_cost << " h_cost: " << currentNode->h_cost << " f: " << currentNode->f() << std::endl;
            // std::cout << " Open list size: " << open_list.size() << std::endl;

            // 如果目标节点已被处理，回溯路径
            if (currentNode->x == gridGoal.first && currentNode->y == gridGoal.second) {
                ROS_INFO("Find a valid path successfully!");
                return reconstructPath(currentNode);  // 通过父节点回溯路径
            }

            
            // if (closed_list[currentNode->x][currentNode->y]) {
            //     continue;  // 如果当前节点已经处理过，则跳过
            // }


            // 将当前节点加入关闭列表
            closed_list[currentNode->x][currentNode->y] = true;

            // 打印关闭列表
            // printClosedList(closed_list);
            // printGcostList(g_cost_list);

            // 获取当前节点的邻居节点（不使用智能指针）
            std::vector<Node> neighbors = getNeighbors(*currentNode, g_cost_list);
            // std::vector<std::shared_ptr<Node>> neighbors = getNeighbors(*currentNode);

            // 遍历邻居节点
            for (auto& neighbor : neighbors) {

                // 将邻居转换为智能指针
                auto neighbor_ptr = std::make_shared<Node>(neighbor);

                // 如果邻居节点在关闭列表中，则跳过
                if (closed_list[neighbor_ptr->x][neighbor_ptr->y]) {
                    continue;
                }

                // 计算该邻居的代价
                double g_cost = currentNode->g_cost + distance(*currentNode, *neighbor_ptr);  // 计算从当前节点到邻居节点的代价
                double h_cost = heuristic({neighbor_ptr->x, neighbor_ptr->y}, gridGoal);

                // 如果新的路径代价更小，更新邻居
                if (g_cost < neighbor_ptr->g_cost || neighbor_ptr->g_cost == 0) {
                    neighbor_ptr->g_cost = g_cost;
                    neighbor_ptr->h_cost = h_cost;
                    neighbor_ptr->parent = currentNode;  // 设置父节点
                    g_cost_list[neighbor_ptr->x][neighbor_ptr->y] = g_cost;

                    // 如果邻居节点不在开放列表中，则将其加入
                    open_list.push(neighbor_ptr);
                    // std::cout << "Neighbor: (" << neighbor_ptr->x << ", " << neighbor_ptr->y << ")"
                    //           << " g_cost: " << neighbor_ptr->g_cost << " h_cost: " << neighbor_ptr->h_cost << " f: " << neighbor_ptr->f() << std::endl;
                    
                }
            }
        }
        // 如果没有找到路径，返回空路径
        ROS_INFO("Can not find a valid path");
        return {};
    }
    
    void reset(){
        num_of_obs_ = 0;
        grid_map_.grid = std::vector<std::vector<int>>(width_, std::vector<int>(height_, 0));
    }
private:

    // 计算启发式代价（使用欧几里得距离）
    double heuristic(const std::pair<int, int>& from, const std::pair<int, int>& to) {
        return std::sqrt(std::pow(from.first - to.first, 2) + std::pow(from.second - to.second, 2));
    }

    // 计算两节点之间的距离（用于邻居代价计算）
    double distance(const Node& a, const Node& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    // 从世界坐标转换到栅格坐标
    std::pair<int, int> worldToGrid(const Eigen::Vector2d& position) {
        int x = std::round((position.x() - map_min_) / grid_resolution_);
        int y = std::round((position.y() - map_min_) / grid_resolution_);
        return {x, y};
    }

    // 从栅格坐标转换到世界坐标（主要用于路径结果显示）
    Eigen::Vector2d gridToWorld(int x, int y) {
        double wx = x * grid_resolution_ + map_min_;
        double wy = y * grid_resolution_ + map_min_;
        return Eigen::Vector2d(wx, wy);
    }

    // 获取当前节点的所有邻居节点
    std::vector<Node> getNeighbors(const Node& current, const std::vector<std::vector<double>>& g_cost_list) {
    // std::vector<std::shared_ptr<Node>> getNeighbors(const Node& current) {
        std::vector<Node> neighbors;
        // std::vector<std::shared_ptr<Node>> neighbors;

        // 八连通邻居
        std::vector<std::pair<int, int>> directions = {
                {1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
        for (const auto& dir : directions) {
            // Step 2: 根据当前节点和方向计算邻居节点的坐标，并将其加入 neighbors

                // your code

            // finish
            // 计算邻居节点的坐标
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;

            // 判断邻居节点是否在地图范围内
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                // 判断该位置是否是障碍物
                if (grid_map_.grid[nx][ny] == 0) {  // 0表示空闲，1表示障碍物
                    
                    // 只计算坐标，延迟计算 g_cost, h_cost, 和 parent
                    double g_cost = g_cost_list[nx][ny];
                    Node neighbor_node(nx, ny, g_cost, 0);  // 初始化代价为0
                    // auto neighbor_node = std::make_shared<Node>(nx, ny, 0, 0);

                    // 将合法的邻居节点添加到邻居列表中
                    neighbors.push_back(neighbor_node);  // 直接存储 Node 对象
                }
            }
        }

        return neighbors;
    }

    // 回溯路径
    std::vector<Eigen::Vector2d> reconstructPath(std::shared_ptr<Node> node) {
        std::vector<Eigen::Vector2d> path;
        while (node) {
            path.push_back(gridToWorld(node->x, node->y));
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        reset();
        return path;
    }

    // 地图数据
    int width_, height_;
    double map_min_, map_max_, grid_resolution_;
    GridMap grid_map_; // 栅格地图，0: 空闲，1: 障碍物
    int num_of_obs_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_planner");
    ros::NodeHandle nh;
    double map_min_, map_max_, grid_resolution_;
    double start_x_, start_y_, goal_x_, goal_y_;
    nh.param("astar_planner/map_min", map_min_, -5.0);
    nh.param("astar_planner/map_max", map_max_, 5.0);
    nh.param("astar_planner/grid_resolution", grid_resolution_, 0.1);
    nh.param("astar_planner/start_x", start_x_, -4.5);
    nh.param("astar_planner/start_y", start_y_, -4.5);
    nh.param("astar_planner/goal_x", goal_x_, 4.5);
    nh.param("astar_planner/goal_y", goal_y_, 4.5);

    // 地图参数
    int grid_width = std::round((map_max_ - map_min_) / grid_resolution_);
    int grid_height = grid_width;

    AStarPlanner planner(grid_width, grid_height, map_min_, map_max_, grid_resolution_);

    // 障碍物订阅
    ros::Subscriber obstacle_sub = nh.subscribe<visualization_msgs::MarkerArray>("obstacles", 1,
                                                                                 [&planner, &grid_resolution_, &map_min_](const visualization_msgs::MarkerArray::ConstPtr& msg) {
                                                                                     for (const auto& marker : msg->markers) {
                                                                                         planner.setObstacle(marker.pose.position.x, marker.pose.position.y, marker.scale.x / 2.0);
                                                                                     }
                                                                                 });



    // 发布路径
    ros::Rate rate(10);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1);
    // 起点和终点参数
    Eigen::Vector2d start(start_x_, start_y_);
    Eigen::Vector2d goal(goal_x_, goal_y_);
    while (ros::ok()) {
        planner.reset();
//        // 等待障碍物加载
//        ros::Duration(1.0).sleep();
        ros::spinOnce();

        // std::cout << "Grid Map before path search:" << std::endl;
        // planner.printGridMap();

        // 执行路径搜索
        std::vector<Eigen::Vector2d> path = planner.findPath(start, goal);

        // 路径可视化
        if (path.empty()){
            continue;
        }
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        for (const auto& point : path) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = 0.0; // 平面路径，z 设置为 0
            path_msg.poses.push_back(pose);
        }
        path_pub.publish(path_msg);
        rate.sleep();
    }

    return 0;
}